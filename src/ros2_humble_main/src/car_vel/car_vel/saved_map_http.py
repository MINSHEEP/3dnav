"""Read-only, identity-bound original PCD transfer; never edits algorithm data."""
from contextlib import contextmanager
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import threading
from urllib.parse import parse_qs, urlsplit

import yaml

MAX_FILE_BYTES = 128 * 1024 * 1024
MAX_POINTS = 4_000_000
DEFAULT_DIRECTORY = '/home/a/3dnav/src/ros2_humble_main/src/navigation/robot_navigation2/pcdmaps'


class SavedMapError(ValueError):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


def _signature(path):
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def pcd_header(stream):
    fields, consumed = {}, 0
    while True:
        line = stream.readline(16385)
        consumed += len(line)
        if not line or len(line) > 16384 or consumed > 65536:
            raise SavedMapError('PCD文件头无效或过长', 422)
        try:
            text = line.decode('ascii').strip()
        except UnicodeError as error:
            raise SavedMapError('PCD文件头不是ASCII', 422) from error
        if not text or text.startswith('#'):
            continue
        key, *values = text.split()
        fields[key.upper()] = values
        if key.upper() == 'DATA':
            try:
                count = int(fields['POINTS'][0])
                width, height = int(fields['WIDTH'][0]), int(fields['HEIGHT'][0])
                mode = values[0].lower()
            except (KeyError, IndexError, ValueError) as error:
                raise SavedMapError('PCD点数/尺寸声明无效', 422) from error
            if count <= 0 or count > MAX_POINTS or width < 1 or height < 1 or width * height != count:
                raise SavedMapError('完整地图点数超出资源限额或尺寸不一致；未抽稀发送', 422)
            if mode not in ('binary', 'ascii', 'binary_compressed'):
                raise SavedMapError('不支持的PCD编码；未回退到简化地图', 422)
            return count, mode


class SavedMapRepository:
    def __init__(self, directory, state):
        self.root = Path(directory).resolve()
        self.state = state
        self.lock = threading.RLock()
        self.verified = {}

    def _active(self, map_name, load_id, stamp_ns=None):
        if (not isinstance(map_name, str) or not re.fullmatch(r'[\w. -]{1,128}', map_name)
                or map_name.startswith('.') or '/' in map_name or '\\' in map_name):
            raise SavedMapError('地图名称无效', 400)
        current = self.state() or {}
        if (self.root / f'.{map_name}.cloud-edit-pending').exists():
            raise SavedMapError('三维地图编辑事务未完成，请先恢复备份')
        expected = dict(map_name=map_name, load_id=load_id)
        if stamp_ns is not None:
            expected['stamp_ns'] = stamp_ns
        if (not load_id or not re.fullmatch(r'\d+', str(current.get('stamp_ns', '')))
                or any(current.get(key) != value for key, value in expected.items())):
            raise SavedMapError('当前加载地图/版本已变化，请重新加载对应地图')
        return dict(current)

    def _path(self, name):
        candidate = self.root / name
        resolved = candidate.resolve()
        if resolved.parent != self.root or candidate.is_symlink():
            raise SavedMapError('地图路径越界或是符号链接', 403)
        if not resolved.is_file():
            raise SavedMapError('完整地图或定位元数据不存在', 404)
        return resolved

    def manifest(self, map_name, load_id, stamp_ns=None):
        stamp_ns = self._active(map_name, load_id, stamp_ns)['stamp_ns']
        with self.lock:
            try:
                path = self._path(map_name + '.pcd')
                metadata_path = self._path(map_name + '.localization.yaml')
                signature = _signature(path), _signature(metadata_path)
                if not 0 < signature[0][2] <= MAX_FILE_BYTES or signature[1][2] > 262144:
                    raise SavedMapError('完整地图超过128MiB或元数据过大；未截断文件', 422)
                cache_key = map_name, signature
                verified = self.verified.get(cache_key)
                if verified is None:
                    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
                    if not isinstance(metadata, dict) or any(metadata.get(key) != value for key, value in {
                        'schema': 'ros2_mid360_localization_map', 'version': 1,
                        'map_name': map_name, 'frame_id': 'map', 'pcd_file': map_name + '.pcd',
                    }.items()):
                        raise SavedMapError('完整地图的定位元数据身份不一致', 422)
                    with path.open('rb') as stream:
                        count, mode = pcd_header(stream)
                        stream.seek(0)
                        digest = hashlib.sha256()
                        for chunk in iter(lambda: stream.read(262144), b''):
                            digest.update(chunk)
                    sha256 = digest.hexdigest()
                    if (metadata.get('point_count') != count or metadata.get('pcd_size_bytes') != signature[0][2]
                            or metadata.get('pcd_sha256') != sha256):
                        raise SavedMapError('PCD点数、文件长度或SHA256与保存元数据不一致', 422)
                    if signature != (_signature(path), _signature(metadata_path)):
                        raise SavedMapError('校验过程中地图文件发生变化，请重试')
                    verified = dict(point_count=count, encoding=mode, sha256=sha256, size_bytes=signature[0][2])
                    self.verified[cache_key] = verified
                    while len(self.verified) > 8:
                        del self.verified[next(iter(self.verified))]
                self._active(map_name, load_id, stamp_ns)
                return dict(verified, map_name=map_name, load_id=load_id, stamp_ns=stamp_ns,
                            frame_id='map', pcd_file=map_name + '.pcd', complete=True)
            except (OSError, yaml.YAMLError, UnicodeError) as error:
                raise SavedMapError('完整地图读取失败：' + str(error), 422) from error

    @contextmanager
    def open_file(self, map_name, load_id, stamp_ns, sha256):
        info = self.manifest(map_name, load_id, stamp_ns)
        if sha256 != info['sha256']:
            raise SavedMapError('完整地图文件版本已变化，请重新取得清单')
        path = self._path(map_name + '.pcd')
        before = _signature(path)
        descriptor = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
        with os.fdopen(descriptor, 'rb') as stream:
            stat = os.fstat(stream.fileno())
            actual = stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns
            # CPython on Windows can report creation time via Path.stat but
            # change time via fstat for the same NTFS handle. File identity,
            # size and mtime must still match; Linux also compares ctime.
            if actual[:4] != before[:4] or (os.name != 'nt' and actual[4] != before[4]):
                raise SavedMapError('地图打开时已改变，请重试')
            # Recheck after opening, including metadata/hash signature changes.
            if self.manifest(map_name, load_id, stamp_ns)['sha256'] != sha256:
                raise SavedMapError('地图文件版本已变化')
            yield stream
            if before != _signature(path):
                raise SavedMapError('地图传输过程中发生变化')


class SavedMapServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, repository):
        self.repository = repository
        self.downloads = threading.BoundedSemaphore(2)
        super().__init__(address, SavedMapHandler)


class SavedMapHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _origin(self):
        origin = self.headers.get('Origin')
        if origin:
            parsed = urlsplit(origin)
            if (parsed.scheme not in ('http', 'https') or parsed.hostname !=
                    urlsplit('http://' + self.headers.get('Host', '')).hostname):
                raise SavedMapError('不允许此页面来源读取完整地图', 403)
        return origin

    def _headers(self, status, size, mime='application/json; charset=utf-8', digest=None):
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(size))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if digest:
            self.send_header('ETag', '"' + digest + '"')
        origin = self.headers.get('Origin')
        if origin:
            self.send_header('Access-Control-Allow-Origin', origin)
            self.send_header('Vary', 'Origin')
        self.end_headers()

    def do_GET(self):
        started = False
        acquired = False
        try:
            self.connection.settimeout(10)
            self._origin()
            url = urlsplit(self.path)
            query = parse_qs(url.query, strict_parsing=True)
            args = {key: query[key][0] for key in ('map_name', 'load_id')}
            args['stamp_ns'] = query.get('stamp_ns', [None])[0]
            if url.path == '/api/saved-map/manifest':
                data = json.dumps(self.server.repository.manifest(**args), ensure_ascii=False).encode()
                self._headers(200, len(data)); started = True
                self.wfile.write(data)
            elif url.path == '/api/saved-map/file':
                if args['stamp_ns'] is None:
                    raise SavedMapError('文件请求必须指定已确认的地图时间戳', 400)
                acquired = self.server.downloads.acquire(blocking=False)
                if not acquired:
                    raise SavedMapError('完整地图传输繁忙，请稍后重试', 429)
                digest = query['sha256'][0]
                with self.server.repository.open_file(**args, sha256=digest) as stream:
                    size = os.fstat(stream.fileno()).st_size
                    self._headers(200, size, 'application/octet-stream', digest); started = True
                    for chunk in iter(lambda: stream.read(262144), b''):
                        self.server.repository._active(**args)
                        self.wfile.write(chunk)
            else:
                raise SavedMapError('只读地图接口不存在', 404)
        except (SavedMapError, KeyError, ValueError, OSError) as error:
            if not started:
                body = json.dumps({'error': str(error)}, ensure_ascii=False).encode()
                self._headers(getattr(error, 'status', 400), len(body))
                try:
                    self.wfile.write(body)
                except OSError:
                    pass
            self.close_connection = True
        finally:
            if acquired:
                self.server.downloads.release()
