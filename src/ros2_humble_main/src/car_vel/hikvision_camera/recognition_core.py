#!/usr/bin/env python3
"""Face/plate registry, matching and task-scoped recognition deduplication.

The module deliberately depends only on the Python standard library.  Model
adapters (OpenCV SFace and HyperLPR3) live at the camera boundary and feed
embeddings/OCR text into this core.  Recognition is informational: every
public result produced here has ``alert=False``.
"""

from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sqlite3
import threading
import time
from typing import Any, Iterable, Iterator, Sequence


RECOGNITION_STATUSES = frozenset({
    'registered', 'unregistered', 'uncertain'})
PLATE_COLORS = frozenset({
    'yellow_black', 'white_black', 'blue_white', 'green_black', 'unknown'})


class CapacityError(ValueError):
    """Raised when a person or vehicle registry reaches its safe limit."""


def utc_now() -> str:
    """Return a stable UTC timestamp suitable for SQLite text ordering."""
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def normalize_plate_number(value: Any) -> str:
    """Normalize OCR/typed plate text without inventing missing characters."""
    text = str(value or '').strip().upper()
    return re.sub(r'[\s·•.\-_]+', '', text)


def normalize_plate_color(value: Any) -> str:
    """Return one of the supported plate color contracts."""
    color = str(value or 'unknown').strip().lower()
    return color if color in PLATE_COLORS else 'unknown'


def _vector(value: Iterable[Any]) -> list[float]:
    result = []
    for item in value:
        number = float(item)
        if not math.isfinite(number):
            raise ValueError('人脸特征包含非有限数值')
        result.append(number)
    if not result:
        raise ValueError('人脸特征不能为空')
    return result


def cosine_similarity(first: Sequence[Any], second: Sequence[Any]) -> float:
    """Return cosine similarity, or zero for invalid/incompatible vectors."""
    try:
        left = [float(item) for item in first]
        right = [float(item) for item in second]
    except (TypeError, ValueError):
        return 0.0
    if not left or len(left) != len(right):
        return 0.0
    if not all(math.isfinite(item) for item in (*left, *right)):
        return 0.0
    left_norm = math.sqrt(sum(item * item for item in left))
    right_norm = math.sqrt(sum(item * item for item in right))
    if left_norm <= 0.0 or right_norm <= 0.0:
        return 0.0
    similarity = sum(a * b for a, b in zip(left, right))
    return max(-1.0, min(1.0, similarity / (left_norm * right_norm)))


def _recognition_result(
        category: str, status: str, confidence: float, *,
        identifier: str = '', display_name: str = '',
        plate_color: str = 'unknown') -> dict[str, Any]:
    if status not in RECOGNITION_STATUSES:
        raise ValueError(f'无效识别状态: {status}')
    return {
        'category': category,
        'status': status,
        'confidence': round(max(0.0, min(1.0, float(confidence))), 6),
        'identifier': str(identifier or ''),
        'display_name': str(display_name or ''),
        'plate_color': normalize_plate_color(plate_color),
        'alert': False,
    }


class RecognitionDatabase:
    """Independent SQLite store for registries and recognition records."""

    def __init__(
            self, path: str | Path | None = None, *,
            person_capacity: int = 50, vehicle_capacity: int = 50):
        default = (
            Path.home() / '.ros' / 'robot_navigation' / 'recognition.db')
        self.path = Path(path or default).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.person_capacity = max(1, int(person_capacity))
        self.vehicle_capacity = max(1, int(vehicle_capacity))
        self._lock = threading.RLock()
        self._closed = False
        self._initialize()

    @contextmanager
    def _connection(self) -> Iterator[sqlite3.Connection]:
        if self._closed:
            raise RuntimeError('识别数据库已经关闭')
        connection = sqlite3.connect(str(self.path), timeout=10.0)
        connection.row_factory = sqlite3.Row
        connection.execute('PRAGMA foreign_keys = ON')
        connection.execute('PRAGMA busy_timeout = 10000')
        try:
            yield connection
            connection.commit()
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()

    def _initialize(self) -> None:
        with self._lock, self._connection() as connection:
            connection.executescript('''
                PRAGMA journal_mode = WAL;
                CREATE TABLE IF NOT EXISTS people (
                    person_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    notes TEXT NOT NULL DEFAULT '',
                    enabled INTEGER NOT NULL DEFAULT 1,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS face_samples (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    person_id TEXT NOT NULL REFERENCES people(person_id)
                        ON DELETE CASCADE,
                    embedding_json TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS vehicles (
                    vehicle_id TEXT PRIMARY KEY,
                    plate_number TEXT NOT NULL UNIQUE,
                    owner TEXT NOT NULL DEFAULT '',
                    plate_color TEXT NOT NULL DEFAULT 'unknown',
                    notes TEXT NOT NULL DEFAULT '',
                    enabled INTEGER NOT NULL DEFAULT 1,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS recognition_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT NOT NULL,
                    track_id TEXT NOT NULL,
                    category TEXT NOT NULL,
                    status TEXT NOT NULL,
                    identifier TEXT NOT NULL DEFAULT '',
                    display_name TEXT NOT NULL DEFAULT '',
                    confidence REAL NOT NULL DEFAULT 0,
                    plate_color TEXT NOT NULL DEFAULT 'unknown',
                    evidence_path TEXT NOT NULL DEFAULT '',
                    occurred_at TEXT NOT NULL,
                    reviewed_status TEXT NOT NULL DEFAULT '',
                    reviewed_by TEXT NOT NULL DEFAULT '',
                    reviewed_at TEXT,
                    UNIQUE(task_id, track_id)
                );
                CREATE TABLE IF NOT EXISTS recognition_audit (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    action TEXT NOT NULL,
                    target_type TEXT NOT NULL,
                    target_id TEXT NOT NULL,
                    detail_json TEXT NOT NULL DEFAULT '{}',
                    operator TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS recognition_events_task_time
                    ON recognition_events(task_id, occurred_at DESC);
                CREATE INDEX IF NOT EXISTS recognition_events_status_time
                    ON recognition_events(status, occurred_at DESC);
            ''')

    def close(self) -> None:
        """Prevent future operations; connections are otherwise short-lived."""
        with self._lock:
            self._closed = True

    @staticmethod
    def _rows(rows: Iterable[sqlite3.Row]) -> list[dict[str, Any]]:
        return [dict(row) for row in rows]

    def _audit(
            self, connection: sqlite3.Connection, action: str,
            target_type: str, target_id: str, detail: dict[str, Any],
            operator: str = '') -> None:
        connection.execute(
            '''INSERT INTO recognition_audit
               (action, target_type, target_id, detail_json, operator,
                created_at) VALUES (?, ?, ?, ?, ?, ?)''',
            (action, target_type, target_id,
             json.dumps(detail, ensure_ascii=False, separators=(',', ':')),
             str(operator or ''), utc_now()))

    def upsert_person(
            self, person_id: str, name: str,
            embeddings: Iterable[Iterable[Any]], *, notes: str = '',
            enabled: bool = True, operator: str = '') -> dict[str, Any]:
        person_id = str(person_id or '').strip()
        name = str(name or '').strip()
        samples = [_vector(item) for item in embeddings]
        if not person_id or not name:
            raise ValueError('人员编号和姓名不能为空')
        if not samples:
            raise ValueError('至少需要一张有效人脸样本')
        dimensions = {len(item) for item in samples}
        if len(dimensions) != 1:
            raise ValueError('同一人员的人脸特征维度必须一致')
        now = utc_now()
        with self._lock, self._connection() as connection:
            existing = connection.execute(
                'SELECT 1 FROM people WHERE person_id = ?',
                (person_id,)).fetchone()
            count = int(connection.execute(
                'SELECT COUNT(*) FROM people').fetchone()[0])
            if existing is None and count >= self.person_capacity:
                raise CapacityError(f'人员库最多{self.person_capacity}人')
            connection.execute(
                '''INSERT INTO people
                   (person_id, name, notes, enabled, created_at, updated_at)
                   VALUES (?, ?, ?, ?, ?, ?)
                   ON CONFLICT(person_id) DO UPDATE SET
                     name=excluded.name, notes=excluded.notes,
                     enabled=excluded.enabled, updated_at=excluded.updated_at''',
                (person_id, name, str(notes or ''), int(bool(enabled)),
                 now, now))
            connection.execute(
                'DELETE FROM face_samples WHERE person_id = ?', (person_id,))
            connection.executemany(
                '''INSERT INTO face_samples
                   (person_id, embedding_json, created_at) VALUES (?, ?, ?)''',
                [(person_id, json.dumps(sample, separators=(',', ':')), now)
                 for sample in samples])
            self._audit(
                connection, 'upsert', 'person', person_id,
                {'name': name, 'sample_count': len(samples)}, operator)
        return self.get_person(person_id) or {}

    def get_person(self, person_id: str) -> dict[str, Any] | None:
        with self._lock, self._connection() as connection:
            row = connection.execute(
                '''SELECT p.*,
                   (SELECT COUNT(*) FROM face_samples s
                    WHERE s.person_id = p.person_id) AS sample_count
                   FROM people p WHERE p.person_id = ?''',
                (str(person_id),)).fetchone()
        return dict(row) if row is not None else None

    def list_people(self) -> list[dict[str, Any]]:
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                '''SELECT p.*,
                   (SELECT COUNT(*) FROM face_samples s
                    WHERE s.person_id = p.person_id) AS sample_count
                   FROM people p ORDER BY p.name, p.person_id''').fetchall()
        return self._rows(rows)

    def delete_person(self, person_id: str, *, operator: str = '') -> bool:
        with self._lock, self._connection() as connection:
            removed = connection.execute(
                'DELETE FROM people WHERE person_id = ?',
                (str(person_id),)).rowcount > 0
            if removed:
                self._audit(
                    connection, 'delete', 'person', str(person_id), {},
                    operator)
        return removed

    def upsert_vehicle(
            self, vehicle_id: str, plate_number: str, *, owner: str = '',
            plate_color: str = 'unknown', notes: str = '',
            enabled: bool = True, operator: str = '') -> dict[str, Any]:
        vehicle_id = str(vehicle_id or '').strip()
        plate = normalize_plate_number(plate_number)
        if not vehicle_id or not plate:
            raise ValueError('车辆编号和车牌号码不能为空')
        now = utc_now()
        with self._lock, self._connection() as connection:
            existing = connection.execute(
                'SELECT 1 FROM vehicles WHERE vehicle_id = ?',
                (vehicle_id,)).fetchone()
            conflict = connection.execute(
                '''SELECT vehicle_id FROM vehicles
                   WHERE plate_number = ? AND vehicle_id <> ?''',
                (plate, vehicle_id)).fetchone()
            if conflict is not None:
                raise ValueError(
                    f'车牌{plate}已属于车辆{conflict["vehicle_id"]}')
            count = int(connection.execute(
                'SELECT COUNT(*) FROM vehicles').fetchone()[0])
            if existing is None and count >= self.vehicle_capacity:
                raise CapacityError(f'车辆库最多{self.vehicle_capacity}辆')
            connection.execute(
                '''INSERT INTO vehicles
                   (vehicle_id, plate_number, owner, plate_color, notes,
                    enabled, created_at, updated_at)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                   ON CONFLICT(vehicle_id) DO UPDATE SET
                     plate_number=excluded.plate_number,
                     owner=excluded.owner, plate_color=excluded.plate_color,
                     notes=excluded.notes, enabled=excluded.enabled,
                     updated_at=excluded.updated_at''',
                (vehicle_id, plate, str(owner or ''),
                 normalize_plate_color(plate_color), str(notes or ''),
                 int(bool(enabled)), now, now))
            self._audit(
                connection, 'upsert', 'vehicle', vehicle_id,
                {'plate_number': plate}, operator)
        return self.get_vehicle(vehicle_id) or {}

    def get_vehicle(self, vehicle_id: str) -> dict[str, Any] | None:
        with self._lock, self._connection() as connection:
            row = connection.execute(
                'SELECT * FROM vehicles WHERE vehicle_id = ?',
                (str(vehicle_id),)).fetchone()
        return dict(row) if row is not None else None

    def list_vehicles(self) -> list[dict[str, Any]]:
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                'SELECT * FROM vehicles ORDER BY plate_number').fetchall()
        return self._rows(rows)

    def delete_vehicle(self, vehicle_id: str, *, operator: str = '') -> bool:
        with self._lock, self._connection() as connection:
            removed = connection.execute(
                'DELETE FROM vehicles WHERE vehicle_id = ?',
                (str(vehicle_id),)).rowcount > 0
            if removed:
                self._audit(
                    connection, 'delete', 'vehicle', str(vehicle_id), {},
                    operator)
        return removed

    def identify_face(
            self, embedding: Iterable[Any], *, quality: float,
            match_threshold: float = 0.45,
            quality_threshold: float = 0.60) -> dict[str, Any]:
        try:
            feature = _vector(embedding)
            quality_value = float(quality)
        except (TypeError, ValueError):
            return _recognition_result('face', 'uncertain', 0.0)
        if quality_value < float(quality_threshold):
            return _recognition_result(
                'face', 'uncertain', max(0.0, quality_value))
        best_score = -1.0
        best_person: sqlite3.Row | None = None
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                '''SELECT p.person_id, p.name, s.embedding_json
                   FROM people p JOIN face_samples s USING(person_id)
                   WHERE p.enabled = 1''').fetchall()
        for row in rows:
            try:
                score = cosine_similarity(
                    feature, json.loads(row['embedding_json']))
            except (TypeError, ValueError, json.JSONDecodeError):
                continue
            if score > best_score:
                best_score = score
                best_person = row
        if best_person is not None and best_score >= float(match_threshold):
            return _recognition_result(
                'face', 'registered', best_score,
                identifier=str(best_person['person_id']),
                display_name=str(best_person['name']))
        return _recognition_result(
            'face', 'unregistered', max(0.0, best_score))

    def identify_plate(
            self, plate_number: Any, *, confidence: float,
            confidence_threshold: float = 0.70,
            plate_color: str = 'unknown') -> dict[str, Any]:
        plate = normalize_plate_number(plate_number)
        try:
            score = float(confidence)
        except (TypeError, ValueError):
            score = 0.0
        color = normalize_plate_color(plate_color)
        if not plate or score < float(confidence_threshold):
            return _recognition_result(
                'plate', 'uncertain', max(0.0, score),
                display_name=plate, plate_color=color)
        with self._lock, self._connection() as connection:
            row = connection.execute(
                '''SELECT vehicle_id, plate_number FROM vehicles
                   WHERE plate_number = ? AND enabled = 1''',
                (plate,)).fetchone()
        if row is not None:
            return _recognition_result(
                'plate', 'registered', score,
                identifier=str(row['vehicle_id']),
                display_name=str(row['plate_number']), plate_color=color)
        return _recognition_result(
            'plate', 'unregistered', score,
            display_name=plate, plate_color=color)

    def record_event(
            self, task_id: str, track_id: str,
            result: dict[str, Any], *, evidence_path: str = '',
            occurred_at: str | None = None) -> int:
        category = str(result.get('category') or '')
        status = str(result.get('status') or '')
        if category not in {'face', 'plate'} or status not in RECOGNITION_STATUSES:
            raise ValueError('识别事件类型或状态无效')
        if bool(result.get('alert', False)):
            raise ValueError('识别事件禁止标记为安全告警')
        with self._lock, self._connection() as connection:
            connection.execute(
                '''INSERT INTO recognition_events
                   (task_id, track_id, category, status, identifier,
                    display_name, confidence, plate_color, evidence_path,
                    occurred_at)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                   ON CONFLICT(task_id, track_id) DO NOTHING''',
                (str(task_id), str(track_id), category, status,
                 str(result.get('identifier') or ''),
                 str(result.get('display_name') or ''),
                 float(result.get('confidence') or 0.0),
                 normalize_plate_color(result.get('plate_color')),
                 str(evidence_path or ''), occurred_at or utc_now()))
            row = connection.execute(
                '''SELECT id FROM recognition_events
                   WHERE task_id = ? AND track_id = ?''',
                (str(task_id), str(track_id))).fetchone()
        return int(row['id'])

    def list_events(
            self, *, task_id: str = '', category: str = '',
            status: str = '', limit: int = 100,
            offset: int = 0) -> list[dict[str, Any]]:
        clauses = []
        values: list[Any] = []
        if task_id:
            clauses.append('task_id = ?')
            values.append(str(task_id))
        if category:
            if category not in {'face', 'plate'}:
                raise ValueError('无效识别类别')
            clauses.append('category = ?')
            values.append(category)
        if status:
            if status not in RECOGNITION_STATUSES:
                raise ValueError('无效识别状态')
            clauses.append('status = ?')
            values.append(status)
        where = ' WHERE ' + ' AND '.join(clauses) if clauses else ''
        values.extend((min(500, max(1, int(limit))), max(0, int(offset))))
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                'SELECT * FROM recognition_events' + where
                + ' ORDER BY occurred_at DESC, id DESC LIMIT ? OFFSET ?',
                values).fetchall()
        return self._rows(rows)

    def task_summary(self, task_id: str) -> dict[str, int]:
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                '''SELECT status, COUNT(*) AS count
                   FROM recognition_events WHERE task_id = ?
                   GROUP BY status''', (str(task_id),)).fetchall()
        counts = {str(row['status']): int(row['count']) for row in rows}
        return {
            'total': sum(counts.values()),
            'registered': counts.get('registered', 0),
            'unregistered': counts.get('unregistered', 0),
            'uncertain': counts.get('uncertain', 0),
        }

    def stats(self) -> dict[str, int]:
        """Return registry/event counters without mixing in safety alerts."""
        with self._lock, self._connection() as connection:
            people = int(connection.execute(
                'SELECT COUNT(*) FROM people').fetchone()[0])
            vehicles = int(connection.execute(
                'SELECT COUNT(*) FROM vehicles').fetchone()[0])
            events = int(connection.execute(
                'SELECT COUNT(*) FROM recognition_events').fetchone()[0])
            pending_review = int(connection.execute(
                """SELECT COUNT(*) FROM recognition_events
                   WHERE reviewed_status = ''""").fetchone()[0])
        return {
            'people': people, 'vehicles': vehicles, 'events': events,
            'pending_review': pending_review,
            'person_capacity': self.person_capacity,
            'vehicle_capacity': self.vehicle_capacity,
        }

    def list_audit(self, *, limit: int = 100, offset: int = 0) -> list[dict[str, Any]]:
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                '''SELECT * FROM recognition_audit
                   ORDER BY created_at DESC, id DESC LIMIT ? OFFSET ?''',
                (min(500, max(1, int(limit))), max(0, int(offset)))).fetchall()
        return self._rows(rows)

    def review_event(
            self, event_id: int, reviewed_status: str, *,
            reviewer: str = '') -> bool:
        if reviewed_status not in RECOGNITION_STATUSES:
            raise ValueError('无效复核状态')
        with self._lock, self._connection() as connection:
            updated = connection.execute(
                '''UPDATE recognition_events SET reviewed_status = ?,
                   reviewed_by = ?, reviewed_at = ? WHERE id = ?''',
                (reviewed_status, str(reviewer or ''), utc_now(),
                 int(event_id))).rowcount > 0
            if updated:
                self._audit(
                    connection, 'review', 'event', str(event_id),
                    {'reviewed_status': reviewed_status}, reviewer)
        return updated

    def cleanup_expired(
            self, *, now_epoch: float | None = None,
            event_retention_days: int = 180,
            audit_retention_days: int = 365,
            registered_evidence_days: int = 90,
            uncertain_evidence_days: int = 30,
            evidence_root: str | Path | None = None,
            operator: str = '') -> dict[str, int]:
        current = time.time() if now_epoch is None else float(now_epoch)
        event_cutoff = datetime.fromtimestamp(
            current - max(1, event_retention_days) * 86400,
            timezone.utc).isoformat(timespec='seconds')
        audit_cutoff = datetime.fromtimestamp(
            current - max(1, audit_retention_days) * 86400,
            timezone.utc).isoformat(timespec='seconds')
        registered_evidence_cutoff = datetime.fromtimestamp(
            current - max(1, registered_evidence_days) * 86400,
            timezone.utc).isoformat(timespec='seconds')
        uncertain_evidence_cutoff = datetime.fromtimestamp(
            current - max(1, uncertain_evidence_days) * 86400,
            timezone.utc).isoformat(timespec='seconds')
        root = Path(evidence_root or (
            Path.home() / '.ros' / 'robot_navigation'
            / 'recognition_evidence')).expanduser().resolve()
        removed_evidence = 0
        with self._lock, self._connection() as connection:
            evidence_rows = connection.execute(
                '''SELECT id, evidence_path FROM recognition_events
                   WHERE evidence_path <> '' AND (
                     (status = 'uncertain' AND occurred_at < ?) OR
                     (status <> 'uncertain' AND occurred_at < ?)
                   )''',
                (uncertain_evidence_cutoff,
                 registered_evidence_cutoff)).fetchall()
            cleared_ids = []
            for row in evidence_rows:
                try:
                    target = Path(row['evidence_path']).expanduser().resolve()
                    target.relative_to(root)
                    if target.is_file():
                        target.unlink()
                        removed_evidence += 1
                    cleared_ids.append(int(row['id']))
                except (OSError, RuntimeError, ValueError):
                    continue
            if cleared_ids:
                connection.executemany(
                    '''UPDATE recognition_events SET evidence_path = ''
                       WHERE id = ?''', [(item,) for item in cleared_ids])
            events = connection.execute(
                'DELETE FROM recognition_events WHERE occurred_at < ?',
                (event_cutoff,)).rowcount
            audits = connection.execute(
                'DELETE FROM recognition_audit WHERE created_at < ?',
                (audit_cutoff,)).rowcount
            self._audit(
                connection, 'cleanup', 'retention', 'recognition', {
                    'events': int(events), 'evidence': removed_evidence,
                    'audits': int(audits),
                }, operator)
        return {
            'events': int(events), 'audits': int(audits),
            'evidence': removed_evidence,
        }


def _bbox(value: Sequence[Any]) -> tuple[float, float, float, float]:
    if len(value) < 4:
        raise ValueError('目标框必须包含四个坐标')
    x1, y1, x2, y2 = [float(item) for item in value[:4]]
    return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)


def _iou(first: Sequence[Any], second: Sequence[Any]) -> float:
    a = _bbox(first)
    b = _bbox(second)
    left, top = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    second_area = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _center_distance(first: Sequence[Any], second: Sequence[Any]) -> float:
    a = _bbox(first)
    b = _bbox(second)
    a_center = ((a[0] + a[2]) / 2.0, (a[1] + a[3]) / 2.0)
    b_center = ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)
    distance = math.hypot(a_center[0] - b_center[0], a_center[1] - b_center[1])
    scale = max(a[2] - a[0], a[3] - a[1],
                b[2] - b[0], b[3] - b[1], 1.0)
    return distance / scale


class TaskTrackDeduplicator:
    """Emit one recognition key per task-local track after stable frames."""

    def __init__(
            self, *, stable_frames: int = 2,
            retention_seconds: float = 120.0, iou_threshold: float = 0.25,
            center_distance_threshold: float = 1.25):
        self.stable_frames = max(1, int(stable_frames))
        self.retention_seconds = max(1.0, float(retention_seconds))
        self.iou_threshold = max(0.0, min(1.0, float(iou_threshold)))
        self.center_distance_threshold = max(
            0.0, float(center_distance_threshold))
        self.task_id = ''
        self._tracks: list[dict[str, Any]] = []
        self._next_ids = {'face': 1, 'plate': 1}

    def start_task(self, task_id: str) -> None:
        """Reset all tracks even when a caller reuses the same tracker."""
        self.task_id = str(task_id or '')
        self._tracks = []
        self._next_ids = {'face': 1, 'plate': 1}

    def _matching_track(
            self, category: str, bbox: Sequence[Any],
            identity_key: str) -> dict[str, Any] | None:
        candidates = [
            item for item in self._tracks
            if item['category'] == category]
        if identity_key:
            identified = [
                item for item in candidates
                if item['identity_key'] == identity_key]
            if identified:
                return max(identified, key=lambda item: item['last_seen'])
        spatial = [
            item for item in candidates
            if _iou(item['bbox'], bbox) >= self.iou_threshold
            or _center_distance(item['bbox'], bbox)
            <= self.center_distance_threshold]
        return max(spatial, key=lambda item: item['last_seen']) if spatial else None

    def observe(
            self, category: str, bbox: Sequence[Any], *,
            identity_key: str = '', now: float | None = None) -> str | None:
        """Return a new track id once; otherwise return ``None``."""
        if not self.task_id:
            raise RuntimeError('必须先调用 start_task()')
        if category not in {'face', 'plate'}:
            raise ValueError('轨迹类别必须是 face 或 plate')
        current = time.monotonic() if now is None else float(now)
        self._tracks = [
            item for item in self._tracks
            if current - float(item['last_seen']) <= self.retention_seconds]
        normalized_box = _bbox(bbox)
        track = self._matching_track(category, normalized_box, identity_key)
        if track is None:
            next_id = self._next_ids[category]
            self._next_ids[category] += 1
            track = {
                'track_id': f'{category}:{next_id}',
                'category': category,
                'bbox': normalized_box,
                'identity_key': str(identity_key or ''),
                'last_seen': current,
                'stable_frames': 1,
                'emitted': False,
            }
            self._tracks.append(track)
        else:
            identity_changed = (
                str(identity_key or '') != str(track['identity_key'] or ''))
            track['bbox'] = normalized_box
            track['last_seen'] = current
            track['identity_key'] = str(identity_key or '')
            if identity_changed and not track['emitted']:
                track['stable_frames'] = 1
            else:
                track['stable_frames'] = int(track['stable_frames']) + 1
        if not track['emitted'] and int(track['stable_frames']) >= self.stable_frames:
            track['emitted'] = True
            return str(track['track_id'])
        return None
