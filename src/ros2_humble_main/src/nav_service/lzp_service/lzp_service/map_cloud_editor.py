"""Ground-preserving saved-map cleanup with preview, backup and recovery.

The caller must hold exclusive edit ownership and stop robot core nodes before
preview/commit/restore. This module never commands a robot or alters live scans.
"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import threading
import time
import uuid

import numpy as np
from PIL import Image

from .pcd_io import _parse_header, _TYPE_CODES
from .terrain_grid import GroundSupportGrid, build_terrain_grid, summarize_terrain_quality
from .terrain_grid_io import load_bound_terrain_grid, write_terrain_grid, create_terrain_grid_metadata


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def sync_directory(path):
    if os.name != 'nt':
        descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def read_records(path):
    path = Path(path)
    if not 0 < path.stat().st_size <= 64 * 1024 * 1024:
        raise ValueError('PCD超过64MiB编辑上限')
    with path.open('rb') as stream:
        header = _parse_header(stream)
        names = header.get('FIELDS', header.get('FIELD', []))
        sizes = list(map(int, header.get('SIZE', [])))
        types = header.get('TYPE', [])
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        n = int(header['POINTS'][0])
        if (not 2000 <= n <= 1_000_000 or not {'x', 'y', 'z'} <= set(names)
                or len(set(names)) != len(names)
                or not len(names) == len(sizes) == len(types) == len(counts)
                or any(c != 1 for c in counts)):
            raise ValueError('PCD字段/数量不支持安全编辑，原文件未修改')
        dtype = np.dtype([(k, _TYPE_CODES[(t, s)]) for k, t, s in zip(names, types, sizes)])
        mode = header['DATA'][0]
        if mode == 'binary':
            raw = stream.read(n * dtype.itemsize + 1)
            if len(raw) != n * dtype.itemsize:
                raise ValueError('PCD内容长度不匹配')
            records = np.frombuffer(raw, dtype=dtype).copy()
        elif mode == 'ascii':
            # Structured load preserves integer/intensity fields as typed data.
            records = np.atleast_1d(np.loadtxt(stream, dtype=dtype, max_rows=n+1))
            if len(records) != n:
                raise ValueError('PCD ASCII记录数不匹配')
        else:
            raise ValueError('仅支持binary/ascii PCD；压缩格式请先另存为受支持格式')
    xyz = np.column_stack([records[k] for k in ('x', 'y', 'z')]).astype(np.float64)
    if not np.isfinite(xyz).all():
        raise ValueError('PCD存在无效坐标，请先修复数据源')
    return records, header, xyz


def write_records(path, records, header):
    lines = ['# saved-map obstacle cleanup; original fields preserved', 'VERSION .7']
    for key in ('FIELDS', 'SIZE', 'TYPE', 'COUNT'):
        values = header.get(key, header.get('FIELD', []) if key == 'FIELDS' else [])
        if values:
            lines.append(key + ' ' + ' '.join(values))
    lines += [f'WIDTH {len(records)}', 'HEIGHT 1',
              'VIEWPOINT ' + ' '.join(header.get('VIEWPOINT', ['0', '0', '0', '1', '0', '0', '0'])),
              f'POINTS {len(records)}', 'DATA binary']
    with Path(path).open('wb') as stream:
        stream.write(('\n'.join(lines) + '\n').encode('ascii'))
        stream.write(records.tobytes())
        stream.flush()
        os.fsync(stream.fileno())


def plan_cleanup(xyz, grid, spec, image, indices, minimum, maximum, *, linked=False):
    """Return an exact deletion mask and conservative PGM copy; no I/O."""
    minimum, maximum = float(minimum), float(maximum)
    if not (math.isfinite(minimum) and math.isfinite(maximum)
            and .15 <= minimum < maximum <= 3.0):
        raise ValueError('相对地面高度需满足0.15m ≤ 下限 < 上限 ≤ 3m')
    w, h = int(spec['width']), int(spec['height'])
    raw = np.asarray(indices)
    if (raw.ndim != 1 or raw.size == 0 or raw.dtype.kind not in 'ui'
            or np.any(raw < 0) or np.any(raw >= w*h)):
        raise ValueError('框选像素索引无效')
    chosen = np.unique(raw.astype(np.int64))
    resolution = float(spec['resolution'])
    if len(chosen) > 10000 or len(chosen) * resolution**2 > 25.0:
        raise ValueError('单次清理不超过10000格/25平方米，请缩小框选')
    x0, y0, yaw = map(float, spec['origin'])
    if abs(yaw) > 1e-8:
        raise ValueError('旋转地图暂不支持三维联动编辑')
    points = np.asarray(xyz, dtype=np.float64)
    cols = np.floor((points[:, 0]-x0)/resolution).astype(np.int64)
    rows = h-1-np.floor((points[:, 1]-y0)/resolution).astype(np.int64)
    in_map = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    selection = np.zeros(w*h, bool)
    selection[chosen] = True
    pixel = np.clip(rows, 0, h-1)*w + np.clip(cols, 0, w-1)
    selected = in_map & selection[pixel]
    tc = np.floor((points[:, 0]-grid.origin_x)/grid.resolution).astype(np.int64)
    tr = np.floor((points[:, 1]-grid.origin_y)/grid.resolution).astype(np.int64)
    inside = (tc >= 0) & (tc < grid.width) & (tr >= 0) & (tr < grid.height)
    gi = np.clip(tr, 0, grid.height-1)*grid.width + np.clip(tc, 0, grid.width-1)
    elevation = np.asarray(grid.elevation).ravel()
    measured = grid.ground_measured
    if measured is None:
        raise ValueError('地图缺少实测地面层，请先重建新版地形')
    ground_known = (np.asarray(measured).ravel() != 0) & np.isfinite(elevation)
    ground_known &= np.asarray(grid.confidence).ravel() >= .35
    step = np.asarray(grid.step_height_m).ravel()
    slope = np.asarray(grid.slope_deg).ravel()
    # Do not mistake a second support surface on stairs/steps for clutter
    # above a lower floor. Uncertain or non-flat cells are intentionally kept.
    flat = np.isfinite(step) & (step <= .08) & np.isfinite(slope) & (np.abs(slope) <= 12)
    trusted = ground_known & flat
    relative = points[:, 2]-elevation[gi]
    # A stored estimate alone cannot authorize removal: actual low returns
    # must still exist in that cell. Never interpolate a floor into a hole.
    floor_hits = inside & trusted[gi] & (np.abs(relative) <= .08)
    floor_count = np.bincount(gi[floor_hits], minlength=grid.width*grid.height)
    usable = inside & trusted[gi] & (floor_count[gi] > 0)
    eligible = selected & ((image.ravel()[pixel] == 0) if linked else True)
    remove = eligible & usable & (relative >= minimum) & (relative <= maximum)
    # Exclusive reasons account for every retained point, not just a generic
    # "protected" total. Preview uses these even when no edits are possible.
    retained = selected & ~remove
    reasons = {}
    for key, condition in (
            ('not_black', ~eligible),
            ('ground_unknown', ~inside | ~ground_known[gi]),
            ('terrain', ~flat[gi]),
            ('no_floor', floor_count[gi] == 0),
            ('ground', np.abs(relative) <= .08),
            ('height', np.ones(len(points), dtype=bool))):
        classified = retained & condition
        reasons[key] = int(np.count_nonzero(classified))
        retained &= ~classified
    result_image = image.copy()
    # No free-space invention: unknown PGM remains unknown, and any remaining
    # body-height point vetoes clearing the associated measured-ground cell.
    collision = usable & ~remove & (relative >= .12) & (relative <= 1.5)
    remaining = np.bincount(gi[collision], minlength=grid.width*grid.height)
    rr, cc = np.divmod(chosen, w)
    wx, wy = x0+(cc+.5)*resolution, y0+(h-rr-.5)*resolution
    gc = np.floor((wx-grid.origin_x)/grid.resolution).astype(np.int64)
    gr = np.floor((wy-grid.origin_y)/grid.resolution).astype(np.int64)
    valid = (gc >= 0) & (gc < grid.width) & (gr >= 0) & (gr < grid.height)
    cell = np.clip(gr, 0, grid.height-1)*grid.width+np.clip(gc, 0, grid.width-1)
    affected = np.bincount(gi[remove], minlength=grid.width*grid.height) > 0
    clear = valid & trusted[cell] & (floor_count[cell] > 0) & (remaining[cell] == 0)
    # Explicit linked erasure may fix a ground-only 2D mislabel without
    # deleting its floor. Legacy 3D cleanup still requires a deleted point.
    if not linked:
        clear &= affected[cell]
    clear &= np.isin(image.ravel()[chosen], [0, 254])
    result_image.ravel()[chosen[clear]] = 254
    relative_samples = relative[selected & inside & ground_known[gi]]
    relative_range = ([float(relative_samples.min()), float(relative_samples.max())]
                      if relative_samples.size else None)
    return {'remove': remove, 'image': result_image, 'reason_counts': reasons,
            'relative_height_range_m': relative_range,
            'selected': selected, 'protected_points': int(np.count_nonzero(selected & ~remove)),
            'cleared_pixels': int(np.count_nonzero(result_image != image))}


class MapCloudEditor:
    """Small interface: preview -> commit(token), or restore latest backup."""
    def __init__(self, maps, clouds, *, load_document=None, write_document=None):
        self.maps, self.clouds = Path(maps), Path(clouds)
        self._lock = threading.RLock()
        self._pending = None
        if load_document is None:
            import yaml
            load_document = lambda p: yaml.safe_load(Path(p).read_text(encoding='utf-8'))
            write_document = lambda p, value: Path(p).write_text(
                yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding='utf-8')
        self.load_document, self.write_document = load_document, write_document

    def paths(self, name):
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', name):
            raise ValueError('无效地图名称')
        return {key: folder/f'{name}{suffix}' for key, folder, suffix in (
            ('image', self.maps, '.pgm'), ('yaml', self.maps, '.yaml'),
            ('pcd', self.clouds, '.pcd'), ('terrain', self.clouds, '.terrain.npz'),
            ('metadata', self.clouds, '.localization.yaml'))}

    def marker(self, name):
        self.paths(name)
        return self.clouds/f'.{name}.cloud-edit-pending'

    def fingerprints(self, paths):
        if any(p.is_symlink() for p in paths.values()):
            raise ValueError('符号链接地图不支持编辑')
        return {key: sha(path) for key, path in paths.items()}

    def preview(self, name, indices, minimum, maximum, *, linked=False):
        with self._lock:
            self._pending = None
            paths = self.paths(name)
            if self.marker(name).exists():
                raise ValueError('上次地图事务未完成，请先恢复备份')
            before = self.fingerprints(paths)
            spec = self.load_document(paths['yaml'])
            metadata = self.load_document(paths['metadata'])
            if (spec.get('image') != f'{name}.pgm' or spec.get('mode', 'trinary') != 'trinary'
                    or spec.get('negate', 0) != 0 or metadata.get('map_name') != name):
                raise ValueError('地图格式或名称不匹配')
            saved = load_bound_terrain_grid(self.clouds, map_name=name,
                localization_metadata=metadata, actual_pcd_sha256=before['pcd'])
            if saved is None:
                raise ValueError('必须先建立有实测地面的新版地形网格')
            records, header, xyz = read_records(paths['pcd'])
            if (metadata.get('schema') != 'ros2_mid360_localization_map' or metadata.get('version') != 1
                    or metadata.get('frame_id') != 'map' or metadata.get('pcd_file') != f'{name}.pcd'
                    or metadata.get('point_count') != len(records)
                    or metadata.get('pcd_size_bytes') != paths['pcd'].stat().st_size):
                raise ValueError('定位元数据的身份、点数或文件长度不一致')
            image = np.asarray(Image.open(paths['image']).convert('L'))
            spec = dict(spec, width=image.shape[1], height=image.shape[0])
            geometry = metadata.get('map_geometry') or {}
            if (geometry.get('width') != spec['width'] or geometry.get('height') != spec['height']
                    or not np.allclose(geometry.get('origin', []), spec['origin'], atol=1e-8)
                    or not math.isclose(float(geometry.get('resolution', 0)), float(spec['resolution']), abs_tol=1e-8)):
                raise ValueError('二维地图与定位元数据几何不匹配')
            plan = plan_cleanup(xyz, saved.grid, spec, image, indices, minimum, maximum, linked=linked)
            count = int(plan['remove'].sum())
            if count > len(records)*.10 or len(records)-count < 2000:
                raise ValueError('删除量超过地图10%或剩余定位点不足2000，请缩小区域')
            committable = count > 0 or plan['cleared_pixels'] > 0
            token = uuid.uuid4().hex if committable else ''
            if committable:
                self._pending = dict(id=token, name=name, expires=time.monotonic()+120,
                    before=before, paths=paths, records=records, header=header, xyz=xyz,
                    metadata=metadata, saved=saved, spec=spec, plan=plan)
            def sample(points):
                # This is only an explicit selected-region preview, not the
                # complete saved-map renderer used by navigation/patrol.
                step = max(1, math.ceil(len(points)/12000))
                return points[::step].astype(np.float32).ravel().tolist()
            return dict(preview_id=token, committable=committable, linked=linked,
                delete_count=count, total_count=len(records),
                protected_points=plan['protected_points'], cleared_pixels=plan['cleared_pixels'],
                reason_counts=plan['reason_counts'], relative_height_range_m=plan['relative_height_range_m'],
                removed_xyz=sample(xyz[plan['remove']]),
                kept_xyz=sample(xyz[plan['selected'] & ~plan['remove']]),
                message=('红色点待删除；二维误黑格待清理，地面保持不变。'
                         if committable else '没有可提交修改；请查看未删除原因和相对高度范围。')
                        + '仅预览，尚未保存；有修改时预览最多120秒有效。')

    @staticmethod
    def _json(path, value):
        temporary = path.with_suffix(path.suffix+'.new')
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        sync_directory(path.parent)

    def _backup_root(self, name):
        self.paths(name)
        return self.clouds/'.cloud-edit-backups'/name

    def _install(self, name, sources, expected, operation):
        targets = self.paths(name)
        if self.fingerprints(targets) != expected:
            raise ValueError('地图文件已变化，预览或备份已失效')
        identity = uuid.uuid4().hex
        root = self._backup_root(name)
        backup = root/identity
        backup.mkdir(parents=True)
        for key, target in targets.items():
            self._replace(target, backup/target.name)
        if self.fingerprints({key: backup/path.name for key, path in targets.items()}) != expected:
            raise ValueError('备份期间原地图变化，未安装任何新文件')
        for directory in (backup, root, root.parent, self.clouds):
            sync_directory(directory)
        latest_path = root/'latest.json'
        previous_latest = json.loads(latest_path.read_text(encoding='utf-8')) if latest_path.exists() else None
        manifest = dict(id=identity, map_name=name, before=expected,
            after=self.fingerprints(sources), operation=operation, status='pending')
        self._json(backup/'manifest.json', manifest)
        self._json(root/'latest.json', {'id': identity})
        self._json(self.marker(name), {'backup_id': identity})
        replaced = []
        try:
            for key, destination in targets.items():
                self._replace(sources[key], destination)
                replaced.append(key)
            manifest['status'] = 'committed'
            self._json(backup/'manifest.json', manifest)
            self.marker(name).unlink()
            sync_directory(self.clouds)
        except Exception:
            errors = []
            for key in reversed(replaced):
                try:
                    self._replace(backup/targets[key].name, targets[key])
                except Exception as error:
                    errors.append(str(error))
            if errors:
                raise ValueError('提交失败且回滚不完整；地图已锁定，需恢复备份：'+'; '.join(errors))
            manifest['status'] = 'rolled_back'
            self._json(backup/'manifest.json', manifest)
            self.marker(name).unlink(missing_ok=True)
            if previous_latest:
                self._json(latest_path, previous_latest)
            else:
                latest_path.unlink(missing_ok=True)
            raise
        return dict(backup_id=identity, backup_path=str(backup),
                    message='地图已保存并备份；请重新加载地图、设置初始定位并预览路线，禁止直接续跑旧任务')

    @staticmethod
    def _replace(source, destination):
        temp = destination.with_name('.'+destination.name+'.cloud-new-'+uuid.uuid4().hex)
        try:
            with Path(source).open('rb') as inp, temp.open('xb') as out:
                shutil.copyfileobj(inp, out)
                out.flush()
                os.fsync(out.fileno())
            os.replace(temp, destination)
            sync_directory(destination.parent)
        finally:
            temp.unlink(missing_ok=True)

    def commit(self, name, token):
        with self._lock:
            pending, self._pending = self._pending, None
            if (not pending or pending['name'] != name or pending['id'] != token
                    or time.monotonic() > pending['expires']):
                raise ValueError('清理预览不存在、过期或已使用，请重新预览')
            paths = pending['paths']
            if self.marker(name).exists() or self.fingerprints(paths) != pending['before']:
                raise ValueError('地图已变化或存在未完成事务，请重新预览/恢复')
            stage = self.clouds/('.'+name+'.cloud-stage-'+uuid.uuid4().hex)
            stage.mkdir()
            installed = None
            try:
                sources = {key: stage/path.name for key, path in paths.items()}
                shutil.copy2(paths['yaml'], sources['yaml'])
                Image.fromarray(pending['plan']['image']).save(sources['image'])
                records = pending['records'][~pending['plan']['remove']]
                xyz = pending['xyz'][~pending['plan']['remove']]
                if pending['plan']['remove'].any():
                    write_records(sources['pcd'], records, pending['header'])
                else:
                    # Ground-only corrections must not rewrite even the PCD
                    # header or original ASCII/field representation.
                    shutil.copy2(paths['pcd'], sources['pcd'])
                spec = pending['spec']
                support = GroundSupportGrid(spec['resolution'], spec['width'], spec['height'],
                    spec['origin'][0], spec['origin'][1],
                    np.flipud(pending['plan']['image'] == 254), spec['origin'][2])
                parameters = pending['saved'].generation.get('parameters')
                if not isinstance(parameters, dict):
                    raise ValueError('地形生成参数缺失')
                terrain = build_terrain_grid(xyz, ground_support=support, **parameters)
                quality = summarize_terrain_quality(terrain, support)
                pcd_hash = sha(sources['pcd'])
                generation = dict(algorithm='clustered_measured_ground_v3',
                    source='manual_obstacle_cleanup', source_pcd_sha256=pcd_hash,
                    parameters=parameters, quality=quality)
                summary = write_terrain_grid(sources['terrain'], map_name=name,
                    frame_id='map', grid=terrain, generation=generation)
                terrain_metadata = create_terrain_grid_metadata(sources['terrain'],
                    summary=summary, source_pcd_sha256=pcd_hash)
                metadata = copy.deepcopy(pending['metadata'])
                metadata.update(point_count=len(records), pcd_size_bytes=sources['pcd'].stat().st_size,
                    pcd_sha256=pcd_hash, terrain_grid=terrain_metadata,
                    cloud_edited_at_utc=datetime.now(timezone.utc).isoformat())
                for key in ('pcd', 'localization_pcd'):
                    if isinstance(metadata.get(key), dict):
                        metadata[key].update(point_count=len(records),
                            size_bytes=metadata['pcd_size_bytes'], sha256=pcd_hash)
                        if pending['plan']['remove'].any():
                            metadata[key]['format'] = 'binary'
                files = metadata.setdefault('files', {})
                files['localization_pcd'] = dict(filename=f'{name}.pcd',
                    size_bytes=metadata['pcd_size_bytes'], sha256=pcd_hash)
                files['terrain_grid'] = dict(filename=f'{name}.terrain.npz',
                    size_bytes=terrain_metadata['size_bytes'], sha256=terrain_metadata['sha256'],
                    source_pcd_sha256=pcd_hash)
                self.write_document(sources['metadata'], metadata)
                # Validate the actual staged metadata as a normal map load will.
                load_bound_terrain_grid(stage, map_name=name,
                    localization_metadata=self.load_document(sources['metadata']), actual_pcd_sha256=pcd_hash)
                reread, _, _ = read_records(sources['pcd'])
                if not np.array_equal(records, reread):
                    raise ValueError('PCD字段回读验证失败')
                installed = self._install(name, sources, pending['before'], 'cleanup')
                return installed
            finally:
                # Stage is generated below clouds with a random exact basename.
                if stage.resolve().parent != self.clouds.resolve() or not stage.name.startswith('.'+name+'.cloud-stage-'):
                    raise ValueError('暂存目录越界，保留现场文件')
                try:
                    shutil.rmtree(stage)
                except OSError:
                    if installed is not None:
                        installed['message'] += f'；暂存清理失败，已保留在{stage}'

    def restore(self, name):
        with self._lock:
            self._pending = None
            root = self._backup_root(name)
            latest = json.loads((root/'latest.json').read_text(encoding='utf-8'))['id']
            if not re.fullmatch(r'[0-9a-f]{32}', latest):
                raise ValueError('备份标识无效')
            backup = root/latest
            manifest = json.loads((backup/'manifest.json').read_text(encoding='utf-8'))
            paths = self.paths(name)
            sources = {key: backup/path.name for key, path in paths.items()}
            if self.fingerprints(sources) != manifest['before']:
                raise ValueError('备份校验失败，未恢复')
            if manifest['status'] == 'committed' and self.fingerprints(paths) != manifest['after']:
                raise ValueError('地图在清理后又被修改，禁止覆盖；请人工核对备份')
            if manifest['status'] not in ('pending', 'committed', 'recovering'):
                raise ValueError('该备份已恢复或事务已回滚')
            if manifest['status'] in ('pending', 'recovering'):
                if (not self.marker(name).exists() or
                        json.loads(self.marker(name).read_text(encoding='utf-8')).get('backup_id') != latest):
                    raise ValueError('待恢复事务标记缺失或不匹配，禁止覆盖地图，请人工核对备份')
            # A pending marker blocks loading if recovery is interrupted.
            self._json(self.marker(name), {'backup_id': latest})
            manifest['status'] = 'recovering'
            self._json(backup/'manifest.json', manifest)
            for key, destination in paths.items():
                self._replace(sources[key], destination)
            manifest['status'] = 'restored'
            self._json(backup/'manifest.json', manifest)
            self.marker(name).unlink(missing_ok=True)
            return dict(message='原地图五件套已恢复；备份仍保留，请重新加载并定位', backup_path=str(backup))
