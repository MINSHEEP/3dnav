"""Persistent storage for inspection templates, task state and evidence."""

from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import threading
from contextlib import contextmanager
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional


TERMINAL_OUTCOMES = {'completed', 'cancelled', 'failed'}
REPORT_STATUSES = {'generating', 'pending_review', 'processing', 'closed'}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def _json_dump(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'))


def _json_load(value: Optional[str], default: Any) -> Any:
    if not value:
        return default
    try:
        return json.loads(value)
    except (TypeError, ValueError):
        return default


class InspectionDatabase:
    """SQLite repository with task updates committed at state boundaries."""

    def __init__(self, db_path: Optional[str] = None):
        default_path = os.path.join(
            os.path.expanduser('~'), '.ros', 'robot_navigation',
            'inspection.db')
        self.db_path = os.path.abspath(os.path.expanduser(db_path or default_path))
        self._lock = threading.RLock()
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.db_path, timeout=10.0)
        connection.row_factory = sqlite3.Row
        connection.execute('PRAGMA foreign_keys = ON')
        connection.execute('PRAGMA busy_timeout = 10000')
        return connection

    @contextmanager
    def _connection(self):
        connection = self._connect()
        try:
            with connection:
                yield connection
        finally:
            connection.close()

    def _initialize(self) -> None:
        with self._lock, self._connection() as connection:
            connection.execute('PRAGMA journal_mode = WAL')
            connection.executescript(
                '''
                CREATE TABLE IF NOT EXISTS inspection_templates (
                    route_id INTEGER PRIMARY KEY,
                    config_json TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS inspection_tasks (
                    id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    route_id INTEGER NOT NULL,
                    route_name TEXT NOT NULL,
                    map_name TEXT NOT NULL,
                    outcome TEXT NOT NULL DEFAULT '',
                    request_json TEXT NOT NULL,
                    template_json TEXT NOT NULL,
                    state_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    started_at TEXT NOT NULL,
                    ended_at TEXT,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS inspection_point_results (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT NOT NULL,
                    loop_number INTEGER NOT NULL,
                    route_point_id INTEGER NOT NULL,
                    point_sequence INTEGER NOT NULL,
                    point_name TEXT NOT NULL,
                    direction TEXT NOT NULL DEFAULT '',
                    result TEXT NOT NULL,
                    detail TEXT NOT NULL DEFAULT '',
                    raw_image TEXT NOT NULL DEFAULT '',
                    annotated_image TEXT NOT NULL DEFAULT '',
                    thermal_raw_image TEXT NOT NULL DEFAULT '',
                    thermal_annotated_image TEXT NOT NULL DEFAULT '',
                    thermal_measurement_json TEXT NOT NULL DEFAULT '{}',
                    created_at TEXT NOT NULL,
                    FOREIGN KEY(task_id) REFERENCES inspection_tasks(id)
                        ON DELETE CASCADE
                );

                CREATE TABLE IF NOT EXISTS inspection_alerts (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT NOT NULL,
                    dedup_key TEXT NOT NULL,
                    loop_number INTEGER NOT NULL,
                    route_point_id INTEGER NOT NULL DEFAULT 0,
                    point_name TEXT NOT NULL DEFAULT '',
                    direction TEXT NOT NULL DEFAULT '',
                    segment_key TEXT NOT NULL DEFAULT '',
                    category TEXT NOT NULL,
                    confidence REAL NOT NULL,
                    measurement_type TEXT NOT NULL DEFAULT 'confidence',
                    severity TEXT NOT NULL DEFAULT 'warning',
                    measurement_json TEXT NOT NULL DEFAULT '{}',
                    first_seen_at TEXT NOT NULL,
                    last_seen_at TEXT NOT NULL,
                    raw_image TEXT NOT NULL DEFAULT '',
                    annotated_image TEXT NOT NULL DEFAULT '',
                    thermal_raw_image TEXT NOT NULL DEFAULT '',
                    thermal_annotated_image TEXT NOT NULL DEFAULT '',
                    map_x REAL,
                    map_y REAL,
                    acknowledged INTEGER NOT NULL DEFAULT 0,
                    acknowledged_at TEXT,
                    FOREIGN KEY(task_id) REFERENCES inspection_tasks(id)
                        ON DELETE CASCADE,
                    UNIQUE(task_id, dedup_key)
                );

                CREATE TABLE IF NOT EXISTS inspection_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT NOT NULL,
                    level TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    message TEXT NOT NULL,
                    data_json TEXT NOT NULL DEFAULT '{}',
                    created_at TEXT NOT NULL,
                    FOREIGN KEY(task_id) REFERENCES inspection_tasks(id)
                        ON DELETE CASCADE
                );

                CREATE TABLE IF NOT EXISTS inspection_reports (
                    task_id TEXT PRIMARY KEY,
                    report_number TEXT NOT NULL UNIQUE,
                    status TEXT NOT NULL DEFAULT 'generating',
                    conclusion TEXT NOT NULL DEFAULT '',
                    reviewer TEXT NOT NULL DEFAULT '',
                    reviewed_at TEXT,
                    closed_at TEXT,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    FOREIGN KEY(task_id) REFERENCES inspection_tasks(id)
                        ON DELETE CASCADE
                );

                CREATE INDEX IF NOT EXISTS idx_inspection_tasks_started
                    ON inspection_tasks(started_at DESC);
                CREATE INDEX IF NOT EXISTS idx_inspection_points_task
                    ON inspection_point_results(task_id, loop_number, id);
                CREATE INDEX IF NOT EXISTS idx_inspection_alerts_task
                    ON inspection_alerts(task_id, acknowledged, id DESC);
                CREATE INDEX IF NOT EXISTS idx_inspection_events_task
                    ON inspection_events(task_id, id);
                CREATE INDEX IF NOT EXISTS idx_inspection_reports_status
                    ON inspection_reports(status, updated_at DESC);
                ''')
            self._ensure_column(
                connection, 'inspection_point_results',
                'thermal_raw_image', "TEXT NOT NULL DEFAULT ''")
            self._ensure_column(
                connection, 'inspection_point_results',
                'thermal_annotated_image', "TEXT NOT NULL DEFAULT ''")
            self._ensure_column(
                connection, 'inspection_point_results',
                'thermal_measurement_json', "TEXT NOT NULL DEFAULT '{}'")
            self._ensure_column(
                connection, 'inspection_alerts', 'measurement_type',
                "TEXT NOT NULL DEFAULT 'confidence'")
            self._ensure_column(
                connection, 'inspection_alerts', 'severity',
                "TEXT NOT NULL DEFAULT 'warning'")
            self._ensure_column(
                connection, 'inspection_alerts', 'measurement_json',
                "TEXT NOT NULL DEFAULT '{}'")
            self._ensure_column(
                connection, 'inspection_alerts', 'thermal_raw_image',
                "TEXT NOT NULL DEFAULT ''")
            self._ensure_column(
                connection, 'inspection_alerts', 'thermal_annotated_image',
                "TEXT NOT NULL DEFAULT ''")
            self._backfill_reports(connection)

    @staticmethod
    def _ensure_column(
            connection: sqlite3.Connection, table: str,
            column: str, definition: str) -> None:
        existing = {
            str(row['name'])
            for row in connection.execute(f'PRAGMA table_info({table})')
        }
        if column not in existing:
            connection.execute(
                f'ALTER TABLE {table} ADD COLUMN {column} {definition}')

    @staticmethod
    def _report_number(task_id: str, started_at: str) -> str:
        date_part = ''.join(character for character in str(started_at)[:10]
                            if character.isdigit())
        if len(date_part) != 8:
            date_part = datetime.now(timezone.utc).strftime('%Y%m%d')
        readable = ''.join(character for character in str(task_id)
                           if character.isalnum()).upper()[:6] or 'REPORT'
        digest = hashlib.sha1(
            str(task_id).encode('utf-8')).hexdigest()[:8].upper()
        return f'IR-{date_part}-{readable}{digest}'

    def _backfill_reports(self, connection: sqlite3.Connection) -> None:
        rows = connection.execute(
            'SELECT id, started_at, outcome FROM inspection_tasks'
        ).fetchall()
        now = utc_now()
        for row in rows:
            status = 'pending_review' if str(row['outcome']) else 'generating'
            connection.execute(
                '''INSERT OR IGNORE INTO inspection_reports(
                       task_id, report_number, status, created_at, updated_at)
                   VALUES (?, ?, ?, ?, ?)''',
                (
                    str(row['id']),
                    self._report_number(str(row['id']), str(row['started_at'])),
                    status, str(row['started_at'] or now), now,
                ),
            )

    def save_template(self, route_id: int, config: Dict[str, Any]) -> Dict[str, Any]:
        now = utc_now()
        payload = dict(config)
        payload['route_id'] = int(route_id)
        payload['updated_at'] = now
        with self._lock, self._connection() as connection:
            connection.execute(
                '''
                INSERT INTO inspection_templates(route_id, config_json, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(route_id) DO UPDATE SET
                    config_json = excluded.config_json,
                    updated_at = excluded.updated_at
                ''',
                (int(route_id), _json_dump(payload), now),
            )
        return payload

    def get_template(self, route_id: int) -> Optional[Dict[str, Any]]:
        with self._lock, self._connection() as connection:
            row = connection.execute(
                'SELECT config_json FROM inspection_templates WHERE route_id = ?',
                (int(route_id),),
            ).fetchone()
        return _json_load(row['config_json'], {}) if row else None

    def create_task(
            self, task_id: str, name: str, route: Dict[str, Any],
            request: Dict[str, Any], template: Dict[str, Any],
            state: Dict[str, Any]) -> None:
        now = utc_now()
        with self._lock, self._connection() as connection:
            connection.execute(
                '''
                INSERT INTO inspection_tasks(
                    id, name, route_id, route_name, map_name, request_json,
                    template_json, state_json, created_at, started_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''',
                (
                    task_id, name, int(route['id']), str(route['name']),
                    str(route['map_name']), _json_dump(request),
                    _json_dump(template), _json_dump(state), now, now, now,
                ),
            )
            connection.execute(
                '''INSERT INTO inspection_reports(
                       task_id, report_number, status, created_at, updated_at)
                   VALUES (?, ?, 'generating', ?, ?)''',
                (task_id, self._report_number(task_id, now), now, now),
            )

    def update_task(
            self, task_id: str, state: Dict[str, Any],
            outcome: Optional[str] = None) -> None:
        now = utc_now()
        ended_at = now if outcome in TERMINAL_OUTCOMES else None
        with self._lock, self._connection() as connection:
            if outcome is None:
                connection.execute(
                    '''UPDATE inspection_tasks
                       SET state_json = ?, updated_at = ? WHERE id = ?''',
                    (_json_dump(state), now, task_id),
                )
            else:
                connection.execute(
                    '''UPDATE inspection_tasks
                       SET state_json = ?, outcome = ?, ended_at = ?,
                           updated_at = ? WHERE id = ?''',
                    (_json_dump(state), outcome, ended_at, now, task_id),
                )
                connection.execute(
                    '''UPDATE inspection_reports SET
                           status = CASE WHEN status = 'generating'
                               THEN 'pending_review' ELSE status END,
                           updated_at = ?
                       WHERE task_id = ?''',
                    (now, task_id),
                )

    @staticmethod
    def _task_from_row(row: sqlite3.Row) -> Dict[str, Any]:
        return {
            'task_id': str(row['id']),
            'name': str(row['name']),
            'route_id': int(row['route_id']),
            'route_name': str(row['route_name']),
            'map_name': str(row['map_name']),
            'outcome': str(row['outcome']),
            'request': _json_load(row['request_json'], {}),
            'template': _json_load(row['template_json'], {}),
            'state': _json_load(row['state_json'], {}),
            'created_at': str(row['created_at']),
            'started_at': str(row['started_at']),
            'ended_at': str(row['ended_at'] or ''),
            'updated_at': str(row['updated_at']),
        }

    def get_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        with self._lock, self._connection() as connection:
            row = connection.execute(
                'SELECT * FROM inspection_tasks WHERE id = ?',
                (task_id,),
            ).fetchone()
        return self._task_from_row(row) if row else None

    def get_active_task(self) -> Optional[Dict[str, Any]]:
        with self._lock, self._connection() as connection:
            row = connection.execute(
                '''SELECT * FROM inspection_tasks
                   WHERE outcome = '' ORDER BY started_at DESC LIMIT 1'''
            ).fetchone()
        return self._task_from_row(row) if row else None

    def list_history(self, limit: int = 50, offset: int = 0) -> List[Dict[str, Any]]:
        limit = min(200, max(1, int(limit or 50)))
        offset = max(0, int(offset or 0))
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                '''
                SELECT t.*,
                    (SELECT COUNT(*) FROM inspection_point_results p
                     WHERE p.task_id = t.id) AS result_count,
                    (SELECT COUNT(*) FROM inspection_alerts a
                     WHERE a.task_id = t.id) AS alert_count,
                    (SELECT COUNT(*) FROM inspection_alerts a
                     WHERE a.task_id = t.id AND a.acknowledged = 0)
                     AS unacknowledged_count
                FROM inspection_tasks t
                ORDER BY t.started_at DESC LIMIT ? OFFSET ?
                ''',
                (limit, offset),
            ).fetchall()
        result = []
        for row in rows:
            task = self._task_from_row(row)
            task.update({
                'result_count': int(row['result_count']),
                'alert_count': int(row['alert_count']),
                'unacknowledged_count': int(row['unacknowledged_count']),
            })
            result.append(task)
        return result

    @staticmethod
    def _report_result(outcome: str, alert_count: int) -> str:
        if not outcome:
            return 'running'
        if outcome == 'completed':
            return 'abnormal' if alert_count else 'normal'
        return outcome

    @staticmethod
    def _duration_seconds(started_at: str, ended_at: str) -> int:
        if not started_at or not ended_at:
            return 0
        try:
            start = datetime.fromisoformat(started_at)
            end = datetime.fromisoformat(ended_at)
        except (TypeError, ValueError):
            return 0
        return max(0, int((end - start).total_seconds()))

    def _report_from_row(self, row: sqlite3.Row) -> Dict[str, Any]:
        task = self._task_from_row(row)
        alert_count = int(row['alert_count'])
        state = task['state']
        completed_points = int(state.get('completed_points') or 0)
        total_points = int(state.get('total_points') or 0)
        completion_rate = (
            round(100.0 * completed_points / total_points, 1)
            if total_points else 0.0
        )
        task.update({
            'report_number': str(row['report_number']),
            'report_status': str(row['report_status']),
            'conclusion': str(row['conclusion'] or ''),
            'reviewer': str(row['reviewer'] or ''),
            'reviewed_at': str(row['reviewed_at'] or ''),
            'closed_at': str(row['closed_at'] or ''),
            'report_updated_at': str(row['report_updated_at']),
            'result_count': int(row['result_count']),
            'alert_count': alert_count,
            'unacknowledged_count': int(row['unacknowledged_count']),
            'navigation_failure_count': int(row['navigation_failure_count']),
            'completed_points': completed_points,
            'total_points': total_points,
            'point_completion_rate': completion_rate,
            'duration_seconds': self._duration_seconds(
                task['started_at'], task['ended_at']),
            'report_result': self._report_result(task['outcome'], alert_count),
            'robot_id': '',
            'robot_name': '本机机器人',
            'robot_group': '默认分组',
            'energy_available': False,
            'video_available': False,
        })
        return task

    @staticmethod
    def _report_select() -> str:
        return '''
            SELECT t.*, r.report_number, r.status AS report_status,
                r.conclusion, r.reviewer, r.reviewed_at, r.closed_at,
                r.updated_at AS report_updated_at,
                (SELECT COUNT(*) FROM inspection_point_results p
                 WHERE p.task_id = t.id) AS result_count,
                (SELECT COUNT(*) FROM inspection_alerts a
                 WHERE a.task_id = t.id) AS alert_count,
                (SELECT COUNT(*) FROM inspection_alerts a
                 WHERE a.task_id = t.id AND a.acknowledged = 0)
                 AS unacknowledged_count,
                (SELECT COUNT(*) FROM inspection_point_results p
                 WHERE p.task_id = t.id AND p.result = 'arrival_failed')
                 AS navigation_failure_count
            FROM inspection_tasks t
            JOIN inspection_reports r ON r.task_id = t.id
        '''

    @staticmethod
    def _report_where(filters: Dict[str, Any]) -> tuple[str, List[Any]]:
        clauses = []
        values: List[Any] = []

        search = str(filters.get('search') or '').strip()
        if search:
            wildcard = f'%{search}%'
            clauses.append(
                '(t.name LIKE ? OR t.route_name LIKE ? '
                'OR r.report_number LIKE ?)')
            values.extend((wildcard, wildcard, wildcard))

        for key, column in (
                ('started_from', 't.started_at'),
                ('started_to', 't.started_at')):
            value = str(filters.get(key) or '').strip()
            if value:
                clauses.append(f'{column} {">=" if key.endswith("from") else "<="} ?')
                values.append(value)

        for key, column in (
                ('map_name', 't.map_name'),
                ('route_name', 't.route_name'),
                ('outcome', 't.outcome')):
            value = str(filters.get(key) or '').strip()
            if value:
                clauses.append(f'{column} = ?')
                values.append(value)

        report_status = str(filters.get('report_status') or '').strip()
        if report_status:
            if report_status not in REPORT_STATUSES:
                raise ValueError('无效的报告状态筛选')
            clauses.append('r.status = ?')
            values.append(report_status)

        result = str(filters.get('result') or '').strip()
        if result == 'normal':
            clauses.append(
                "t.outcome = 'completed' AND NOT EXISTS ("
                'SELECT 1 FROM inspection_alerts a WHERE a.task_id = t.id)')
        elif result == 'abnormal':
            clauses.append(
                "t.outcome = 'completed' AND EXISTS ("
                'SELECT 1 FROM inspection_alerts a WHERE a.task_id = t.id)')
        elif result in {'failed', 'cancelled'}:
            clauses.append('t.outcome = ?')
            values.append(result)
        elif result == 'running':
            clauses.append("t.outcome = ''")
        elif result:
            raise ValueError('无效的任务结果筛选')

        if filters.get('has_alerts') is True:
            clauses.append(
                'EXISTS (SELECT 1 FROM inspection_alerts a '
                'WHERE a.task_id = t.id)')
        if filters.get('unacknowledged_only') is True:
            clauses.append(
                'EXISTS (SELECT 1 FROM inspection_alerts a '
                'WHERE a.task_id = t.id AND a.acknowledged = 0)')

        return (' WHERE ' + ' AND '.join(clauses) if clauses else ''), values

    def query_reports(
            self, filters: Optional[Dict[str, Any]] = None,
            limit: int = 50, offset: int = 0
            ) -> Dict[str, Any]:
        filters = filters or {}
        limit = min(200, max(1, int(limit or 50)))
        offset = max(0, int(offset or 0))
        where_sql, values = self._report_where(filters)
        from_sql = ''' FROM inspection_tasks t
                       JOIN inspection_reports r ON r.task_id = t.id '''
        with self._lock, self._connection() as connection:
            total = int(connection.execute(
                'SELECT COUNT(*)' + from_sql + where_sql,
                values,
            ).fetchone()[0])
            summary_row = connection.execute(
                '''SELECT COUNT(*) AS report_count,
                       SUM(CASE WHEN t.outcome = 'completed' THEN 1 ELSE 0 END)
                           AS successful_tasks,
                       SUM(CASE WHEN t.outcome <> '' THEN 1 ELSE 0 END)
                           AS terminal_tasks,
                       SUM((SELECT COUNT(*) FROM inspection_alerts a
                            WHERE a.task_id = t.id)) AS alert_count,
                       SUM((SELECT COUNT(*) FROM inspection_alerts a
                            WHERE a.task_id = t.id AND a.acknowledged = 0))
                           AS unacknowledged_count,
                       SUM((SELECT COUNT(*) FROM inspection_point_results p
                            WHERE p.task_id = t.id
                              AND p.result = 'arrival_failed'))
                           AS navigation_failures,
                       AVG(CASE WHEN t.ended_at IS NOT NULL
                           THEN (julianday(t.ended_at) - julianday(t.started_at))
                                * 86400.0 END) AS average_duration_seconds
                   ''' + from_sql + where_sql,
                values,
            ).fetchone()
            rows = connection.execute(
                self._report_select() + where_sql
                + ' ORDER BY t.started_at DESC LIMIT ? OFFSET ?',
                [*values, limit, offset],
            ).fetchall()

        terminal_tasks = int(summary_row['terminal_tasks'] or 0)
        successful_tasks = int(summary_row['successful_tasks'] or 0)
        return {
            'reports': [self._report_from_row(row) for row in rows],
            'total': total,
            'summary': {
                'report_count': int(summary_row['report_count'] or 0),
                'successful_tasks': successful_tasks,
                'completion_rate': round(
                    100.0 * successful_tasks / terminal_tasks, 1
                ) if terminal_tasks else 0.0,
                'alert_count': int(summary_row['alert_count'] or 0),
                'unacknowledged_count': int(
                    summary_row['unacknowledged_count'] or 0),
                'navigation_failures': int(
                    summary_row['navigation_failures'] or 0),
                'average_duration_seconds': int(
                    summary_row['average_duration_seconds'] or 0),
                'energy_available': False,
            },
        }

    def get_report(self, task_id: str) -> Optional[Dict[str, Any]]:
        with self._lock, self._connection() as connection:
            row = connection.execute(
                self._report_select() + ' WHERE t.id = ?',
                (str(task_id),),
            ).fetchone()
        if row is None:
            return None
        report = self._report_from_row(row)
        details = self.get_task_details(str(task_id)) or {}
        report['point_results'] = details.get('point_results', [])
        report['alerts'] = details.get('alerts', [])
        report['events'] = details.get('events', [])
        return report

    def review_report(
            self, task_id: str, report_status: str,
            conclusion: str, reviewer: str) -> Dict[str, Any]:
        task_id = str(task_id).strip()
        report_status = str(report_status).strip()
        conclusion = str(conclusion).strip()
        reviewer = str(reviewer).strip()
        if report_status not in {'pending_review', 'processing', 'closed'}:
            raise ValueError('报告状态必须为待复核、处理中或已关闭')
        if not reviewer:
            raise ValueError('请填写复核人')

        now = utc_now()
        with self._lock, self._connection() as connection:
            row = connection.execute(
                '''SELECT t.outcome, r.status,
                       (SELECT COUNT(*) FROM inspection_alerts a
                        WHERE a.task_id = t.id AND a.acknowledged = 0)
                        AS unacknowledged_count
                   FROM inspection_tasks t
                   JOIN inspection_reports r ON r.task_id = t.id
                   WHERE t.id = ?''',
                (task_id,),
            ).fetchone()
            if row is None:
                raise ValueError('巡检报告不存在')
            if not str(row['outcome']):
                raise ValueError('任务尚未结束，报告仍在生成中')
            if str(row['status']) == 'closed' and report_status != 'closed':
                raise ValueError('已关闭报告暂不支持重新打开')
            if report_status == 'closed':
                if not conclusion:
                    raise ValueError('关闭报告前必须填写总体结论')
                if int(row['unacknowledged_count'] or 0):
                    raise ValueError('关闭报告前必须确认全部告警')

            closed_at = now if report_status == 'closed' else None
            connection.execute(
                '''UPDATE inspection_reports SET status = ?, conclusion = ?,
                       reviewer = ?, reviewed_at = ?, closed_at = ?,
                       updated_at = ? WHERE task_id = ?''',
                (
                    report_status, conclusion, reviewer, now,
                    closed_at, now, task_id,
                ),
            )
            event_type = (
                'report_closed' if report_status == 'closed'
                else 'report_reviewed'
            )
            connection.execute(
                '''INSERT INTO inspection_events(
                       task_id, level, event_type, message,
                       data_json, created_at)
                   VALUES (?, 'info', ?, ?, ?, ?)''',
                (
                    task_id, event_type,
                    '巡检报告已关闭' if report_status == 'closed'
                    else '巡检报告复核信息已更新',
                    _json_dump({
                        'report_status': report_status,
                        'reviewer': reviewer,
                    }),
                    now,
                ),
            )

        report = self.get_report(task_id)
        if report is None:
            raise ValueError('巡检报告不存在')
        return report

    def add_point_result(self, task_id: str, result: Dict[str, Any]) -> int:
        with self._lock, self._connection() as connection:
            cursor = connection.execute(
                '''
                INSERT INTO inspection_point_results(
                    task_id, loop_number, route_point_id, point_sequence,
                    point_name, direction, result, detail, raw_image,
                    annotated_image, thermal_raw_image,
                    thermal_annotated_image, thermal_measurement_json,
                    created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''',
                (
                    task_id, int(result.get('loop_number', 0)),
                    int(result.get('route_point_id', 0)),
                    int(result.get('point_sequence', -1)),
                    str(result.get('point_name', '')),
                    str(result.get('direction', '')),
                    str(result.get('result', '')),
                    str(result.get('detail', '')),
                    str(result.get('raw_image', '')),
                    str(result.get('annotated_image', '')),
                    str(result.get('thermal_raw_image', '')),
                    str(result.get('thermal_annotated_image', '')),
                    _json_dump(result.get('thermal_measurement') or {}),
                    str(result.get('created_at') or utc_now()),
                ),
            )
            return int(cursor.lastrowid)

    def add_event(
            self, task_id: str, event_type: str, message: str,
            level: str = 'info', data: Optional[Dict[str, Any]] = None) -> int:
        with self._lock, self._connection() as connection:
            cursor = connection.execute(
                '''INSERT INTO inspection_events(
                       task_id, level, event_type, message, data_json, created_at)
                   VALUES (?, ?, ?, ?, ?, ?)''',
                (task_id, level, event_type, message,
                 _json_dump(data or {}), utc_now()),
            )
            return int(cursor.lastrowid)

    def upsert_alert(self, task_id: str, alert: Dict[str, Any]) -> int:
        now = str(alert.get('occurred_at') or utc_now())
        confidence = float(alert.get('confidence') or 0.0)
        dedup_key = str(alert['dedup_key'])
        measurement_type = str(
            alert.get('measurement_type') or 'confidence')
        severity = str(alert.get('severity') or 'warning')
        measurement = dict(alert.get('measurement') or {})
        if measurement_type == 'temperature':
            for key in (
                    'max_temperature_c', 'threshold_c', 'unit',
                    'x_normalized', 'y_normalized', 'temperature_source'):
                if key in alert and key not in measurement:
                    measurement[key] = alert[key]
        measurement_json = _json_dump(measurement)
        with self._lock, self._connection() as connection:
            row = connection.execute(
                '''SELECT id, confidence, measurement_type, measurement_json
                   FROM inspection_alerts
                   WHERE task_id = ? AND dedup_key = ?''',
                (task_id, dedup_key),
            ).fetchone()
            if row is None:
                cursor = connection.execute(
                    '''
                    INSERT INTO inspection_alerts(
                        task_id, dedup_key, loop_number, route_point_id,
                        point_name, direction, segment_key, category,
                        confidence, measurement_type, severity, measurement_json,
                        first_seen_at, last_seen_at, raw_image,
                        annotated_image, thermal_raw_image,
                        thermal_annotated_image, map_x, map_y)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ''',
                    (
                        task_id, dedup_key, int(alert.get('loop_number', 0)),
                        int(alert.get('route_point_id', 0)),
                        str(alert.get('point_name', '')),
                        str(alert.get('direction', '')),
                        str(alert.get('segment_key', '')),
                        str(alert.get('category', 'unknown')), confidence,
                        measurement_type, severity, measurement_json,
                        now, now, str(alert.get('raw_image', '')),
                        str(alert.get('annotated_image', '')),
                        str(alert.get('thermal_raw_image', '')),
                        str(alert.get('thermal_annotated_image', '')),
                        alert.get('map_x'), alert.get('map_y'),
                    ),
                )
                return int(cursor.lastrowid)
            alert_id = int(row['id'])
            previous_measurement = _json_load(row['measurement_json'], {})
            if measurement_type == 'temperature':
                current_value = float(measurement.get('max_temperature_c', 0.0))
                previous_value = float(
                    previous_measurement.get('max_temperature_c', 0.0))
                replace_evidence = (
                    str(row['measurement_type']) != 'temperature'
                    or current_value >= previous_value)
            else:
                replace_evidence = (
                    str(row['measurement_type']) != measurement_type
                    or confidence >= float(row['confidence']))
            if replace_evidence:
                connection.execute(
                    '''UPDATE inspection_alerts SET confidence = ?,
                       measurement_type = ?, severity = ?, measurement_json = ?,
                       last_seen_at = ?, raw_image = ?, annotated_image = ?,
                       thermal_raw_image = ?, thermal_annotated_image = ?,
                       map_x = COALESCE(?, map_x), map_y = COALESCE(?, map_y)
                       WHERE id = ?''',
                    (
                        confidence, measurement_type, severity, measurement_json,
                        now, str(alert.get('raw_image', '')),
                        str(alert.get('annotated_image', '')),
                        str(alert.get('thermal_raw_image', '')),
                        str(alert.get('thermal_annotated_image', '')),
                        alert.get('map_x'), alert.get('map_y'), alert_id,
                    ),
                )
            else:
                connection.execute(
                    'UPDATE inspection_alerts SET last_seen_at = ? WHERE id = ?',
                    (now, alert_id),
                )
            return alert_id

    def acknowledge_alert(self, alert_id: int) -> bool:
        with self._lock, self._connection() as connection:
            cursor = connection.execute(
                '''UPDATE inspection_alerts SET acknowledged = 1,
                   acknowledged_at = ? WHERE id = ?''',
                (utc_now(), int(alert_id)),
            )
            return cursor.rowcount > 0

    def get_alerts(self, task_id: str) -> List[Dict[str, Any]]:
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                '''SELECT * FROM inspection_alerts
                   WHERE task_id = ? ORDER BY acknowledged, last_seen_at DESC''',
                (task_id,),
            ).fetchall()
        return [
            {
                **dict(row),
                # The legacy NOT NULL column remains compatible with old DBs;
                # callers see no invented probability for geometric evidence.
                'confidence': (None if row['measurement_type'] == 'rule_evidence'
                               else row['confidence']),
                'measurement': _json_load(row['measurement_json'], {}),
            }
            for row in rows
        ]

    def get_task_details(self, task_id: str) -> Optional[Dict[str, Any]]:
        task = self.get_task(task_id)
        if task is None:
            return None
        with self._lock, self._connection() as connection:
            point_rows = connection.execute(
                '''SELECT * FROM inspection_point_results
                   WHERE task_id = ? ORDER BY id''',
                (task_id,),
            ).fetchall()
            event_rows = connection.execute(
                '''SELECT * FROM inspection_events
                   WHERE task_id = ? ORDER BY id''',
                (task_id,),
            ).fetchall()
        task['point_results'] = [
            {
                **dict(row),
                'thermal_measurement': _json_load(
                    row['thermal_measurement_json'], {}),
            }
            for row in point_rows
        ]
        task['alerts'] = self.get_alerts(task_id)
        task['events'] = [
            {**dict(row), 'data': _json_load(row['data_json'], {})}
            for row in event_rows
        ]
        return task

    def delete_task(self, task_id: str) -> bool:
        with self._lock, self._connection() as connection:
            cursor = connection.execute(
                'DELETE FROM inspection_tasks WHERE id = ?',
                (task_id,),
            )
            return cursor.rowcount > 0

    def delete_completed_before(self, cutoff: str) -> List[str]:
        """Delete terminal task/report rows older than an ISO-8601 cutoff."""
        cutoff = str(cutoff or '').strip()
        if not cutoff:
            raise ValueError('巡检报告清理截止时间不能为空')
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                '''SELECT id FROM inspection_tasks
                   WHERE ended_at IS NOT NULL AND ended_at < ?''',
                (cutoff,)).fetchall()
            task_ids = [str(row['id']) for row in rows]
            if task_ids:
                connection.executemany(
                    'DELETE FROM inspection_tasks WHERE id = ?',
                    [(task_id,) for task_id in task_ids])
        return task_ids
