"""Tests for inspection persistence and alert de-duplication."""

import sqlite3

from lzp_service.inspection_database import InspectionDatabase


def _task_state(task_id='task-1'):
    return {
        'task_id': task_id,
        'state': 'preparing',
        'current_loop': 1,
        'loop_count': 2,
    }


def test_template_task_history_and_delete(tmp_path):
    database = InspectionDatabase(str(tmp_path / 'inspection.db'))
    template = database.save_template(7, {
        'checkpoints': [{
            'route_point_id': 71,
            'directions': ['front', 'left'],
            'zoom_ratio': None,
            'save_images': True,
        }],
    })
    assert database.get_template(7) == template

    database.create_task(
        'task-1', '夜间巡检',
        {'id': 7, 'name': '一楼路线', 'map_name': 'floor1'},
        {'loop_count': 2, 'bidirectional': True},
        template,
        _task_state(),
    )
    assert database.get_active_task()['task_id'] == 'task-1'

    database.add_point_result('task-1', {
        'loop_number': 1,
        'route_point_id': 71,
        'point_sequence': 0,
        'point_name': '入口',
        'direction': 'front',
        'result': 'completed',
    })
    database.add_event('task-1', 'started', '任务已开始')

    database.update_task(
        'task-1', {**_task_state(), 'state': 'completed'}, 'completed')
    assert database.get_active_task() is None
    history = database.list_history()
    assert history[0]['outcome'] == 'completed'
    assert history[0]['result_count'] == 1
    details = database.get_task_details('task-1')
    assert details['point_results'][0]['point_name'] == '入口'
    assert details['events'][0]['event_type'] == 'started'
    assert database.delete_task('task-1') is True
    assert database.get_task('task-1') is None


def test_completed_reports_expire_after_configured_cutoff(tmp_path):
    database = InspectionDatabase(str(tmp_path / 'inspection.db'))
    template = database.save_template(7, {'checkpoints': []})
    for task_id in ('old-task', 'new-task', 'active-task'):
        database.create_task(
            task_id, task_id,
            {'id': 7, 'name': 'route', 'map_name': 'map'},
            {'loop_count': 1}, template, _task_state(task_id))
    database.update_task(
        'old-task', {**_task_state('old-task'), 'state': 'completed'}, 'completed')
    database.update_task(
        'new-task', {**_task_state('new-task'), 'state': 'completed'}, 'completed')
    with database._lock, database._connection() as connection:
        connection.execute(
            "UPDATE inspection_tasks SET ended_at = '2024-01-01T00:00:00+00:00' WHERE id = 'old-task'")
        connection.execute(
            "UPDATE inspection_tasks SET ended_at = '2026-01-01T00:00:00+00:00' WHERE id = 'new-task'")

    removed = database.delete_completed_before('2025-01-01T00:00:00+00:00')
    assert removed == ['old-task']
    assert database.get_task('old-task') is None
    assert database.get_task('new-task') is not None
    assert database.get_task('active-task') is not None


def test_alert_upsert_keeps_best_frame_and_acknowledges(tmp_path):
    database = InspectionDatabase(str(tmp_path / 'inspection.db'))
    template = database.save_template(7, {'checkpoints': []})
    database.create_task(
        'task-1', '巡检',
        {'id': 7, 'name': '路线', 'map_name': 'map'},
        {'loop_count': 1}, template, _task_state(),
    )
    base = {
        'dedup_key': '1:71:front:person',
        'loop_number': 1,
        'route_point_id': 71,
        'point_name': '入口',
        'direction': 'front',
        'category': 'person',
    }
    alert_id = database.upsert_alert('task-1', {
        **base,
        'confidence': 0.7,
        'raw_image': 'raw-a.jpg',
        'annotated_image': 'ann-a.jpg',
    })
    assert database.upsert_alert('task-1', {
        **base,
        'confidence': 0.6,
        'raw_image': 'raw-b.jpg',
        'annotated_image': 'ann-b.jpg',
    }) == alert_id
    assert database.get_alerts('task-1')[0]['raw_image'] == 'raw-a.jpg'
    database.upsert_alert('task-1', {
        **base,
        'confidence': 0.9,
        'raw_image': 'raw-c.jpg',
        'annotated_image': 'ann-c.jpg',
    })
    alert = database.get_alerts('task-1')[0]
    assert alert['confidence'] == 0.9
    assert alert['raw_image'] == 'raw-c.jpg'
    assert database.acknowledge_alert(alert_id) is True
    assert database.get_alerts('task-1')[0]['acknowledged'] == 1


def test_temperature_alert_keeps_peak_and_thermal_evidence(tmp_path):
    database = InspectionDatabase(str(tmp_path / 'inspection.db'))
    template = database.save_template(7, {'checkpoints': []})
    database.create_task(
        'thermal-task', '高温巡检',
        {'id': 7, 'name': '路线', 'map_name': 'map'},
        {'loop_count': 1}, template, _task_state('thermal-task'),
    )
    base = {
        'dedup_key': 'thermal:event-1',
        'loop_number': 1,
        'category': 'high_temperature',
        'measurement_type': 'temperature',
        'severity': 'alarm',
    }
    alert_id = database.upsert_alert('thermal-task', {
        **base,
        'measurement': {'max_temperature_c': 72.4, 'threshold_c': 60.0,
                        'unit': 'C'},
        'thermal_raw_image': 'thermal-raw-a.jpg',
        'thermal_annotated_image': 'thermal-ann-a.jpg',
    })
    assert database.upsert_alert('thermal-task', {
        **base,
        'measurement': {'max_temperature_c': 68.0, 'threshold_c': 60.0,
                        'unit': 'C'},
        'thermal_raw_image': 'thermal-raw-b.jpg',
        'thermal_annotated_image': 'thermal-ann-b.jpg',
    }) == alert_id
    alert = database.get_alerts('thermal-task')[0]
    assert alert['measurement_type'] == 'temperature'
    assert alert['confidence'] == 0.0
    assert alert['measurement']['max_temperature_c'] == 72.4
    assert alert['measurement']['threshold_c'] == 60.0
    assert alert['thermal_raw_image'] == 'thermal-raw-a.jpg'


def test_point_result_round_trips_thermal_measurement(tmp_path):
    database = InspectionDatabase(str(tmp_path / 'inspection.db'))
    template = database.save_template(7, {'checkpoints': []})
    database.create_task(
        'thermal-point', '热成像点检',
        {'id': 7, 'name': '路线', 'map_name': 'map'},
        {'loop_count': 1}, template, _task_state('thermal-point'),
    )
    database.add_point_result('thermal-point', {
        'loop_number': 1, 'route_point_id': 71, 'point_sequence': 0,
        'point_name': '入口', 'result': 'completed',
        'thermal_raw_image': 'thermal-raw.jpg',
        'thermal_annotated_image': 'thermal-ann.jpg',
        'thermal_measurement': {
            'max_temperature_c': 61.2, 'threshold_c': 60.0,
        },
    })
    point = database.get_task_details('thermal-point')['point_results'][0]
    assert point['thermal_raw_image'] == 'thermal-raw.jpg'
    assert point['thermal_measurement']['max_temperature_c'] == 61.2


def test_report_query_review_and_close_rules(tmp_path):
    database = InspectionDatabase(str(tmp_path / 'inspection.db'))
    template = database.save_template(7, {'checkpoints': []})
    database.create_task(
        'task-report-1', '夜间巡检',
        {'id': 7, 'name': '一楼路线', 'map_name': 'floor1'},
        {'loop_count': 1}, template,
        {**_task_state('task-report-1'), 'total_points': 2,
         'completed_points': 2},
    )
    database.add_point_result('task-report-1', {
        'loop_number': 1,
        'route_point_id': 71,
        'point_sequence': 0,
        'point_name': '入口',
        'direction': 'front',
        'result': 'arrival_failed',
    })
    alert_id = database.upsert_alert('task-report-1', {
        'dedup_key': '1:71:front:person',
        'loop_number': 1,
        'route_point_id': 71,
        'point_name': '入口',
        'direction': 'front',
        'category': 'person',
        'confidence': 0.91,
    })
    database.update_task(
        'task-report-1',
        {**_task_state('task-report-1'), 'state': 'completed',
         'total_points': 2, 'completed_points': 2},
        'completed',
    )

    queried = database.query_reports({
        'result': 'abnormal',
        'unacknowledged_only': True,
    })
    assert queried['total'] == 1
    report = queried['reports'][0]
    assert report['report_number'].startswith('IR-')
    assert report['report_status'] == 'pending_review'
    assert report['report_result'] == 'abnormal'
    assert report['point_completion_rate'] == 100.0
    assert report['navigation_failure_count'] == 1
    assert queried['summary']['alert_count'] == 1
    assert queried['summary']['unacknowledged_count'] == 1

    try:
        database.review_report(
            'task-report-1', 'closed', '发现人员进入', '操作员')
    except ValueError as error:
        assert '确认全部告警' in str(error)
    else:
        raise AssertionError('unacknowledged alerts must prevent closing')

    processing = database.review_report(
        'task-report-1', 'processing', '正在现场复核', '操作员')
    assert processing['report_status'] == 'processing'
    assert database.acknowledge_alert(alert_id) is True
    closed = database.review_report(
        'task-report-1', 'closed', '异常已确认并完成处理', '操作员')
    assert closed['report_status'] == 'closed'
    assert closed['closed_at']
    assert closed['events'][-1]['event_type'] == 'report_closed'
    assert database.query_reports({'report_status': 'closed'})['total'] == 1


def test_existing_task_database_backfills_reports(tmp_path):
    db_path = tmp_path / 'inspection.db'
    connection = sqlite3.connect(db_path)
    connection.execute(
        '''CREATE TABLE inspection_tasks (
               id TEXT PRIMARY KEY, name TEXT NOT NULL, route_id INTEGER NOT NULL,
               route_name TEXT NOT NULL, map_name TEXT NOT NULL,
               outcome TEXT NOT NULL DEFAULT '', request_json TEXT NOT NULL,
               template_json TEXT NOT NULL, state_json TEXT NOT NULL,
               created_at TEXT NOT NULL, started_at TEXT NOT NULL,
               ended_at TEXT, updated_at TEXT NOT NULL)''')
    connection.execute(
        '''INSERT INTO inspection_tasks VALUES (
               'legacy-task', '旧任务', 7, '旧路线', 'floor1', 'completed',
               '{}', '{}', '{}', '2026-08-01T08:00:00+00:00',
               '2026-08-01T08:00:00+00:00',
               '2026-08-01T08:30:00+00:00',
               '2026-08-01T08:30:00+00:00')''')
    connection.commit()
    connection.close()

    database = InspectionDatabase(str(db_path))
    report = database.get_report('legacy-task')
    assert report is not None
    assert report['report_number'].startswith('IR-20260801-')
    assert report['report_status'] == 'pending_review'


def test_existing_point_and_alert_tables_gain_thermal_columns(tmp_path):
    db_path = tmp_path / 'inspection.db'
    connection = sqlite3.connect(db_path)
    connection.executescript(
        '''CREATE TABLE inspection_point_results (
               id INTEGER PRIMARY KEY AUTOINCREMENT, task_id TEXT NOT NULL,
               loop_number INTEGER NOT NULL, route_point_id INTEGER NOT NULL,
               point_sequence INTEGER NOT NULL, point_name TEXT NOT NULL,
               direction TEXT NOT NULL DEFAULT '', result TEXT NOT NULL,
               detail TEXT NOT NULL DEFAULT '', raw_image TEXT NOT NULL DEFAULT '',
               annotated_image TEXT NOT NULL DEFAULT '', created_at TEXT NOT NULL);
           CREATE TABLE inspection_alerts (
               id INTEGER PRIMARY KEY AUTOINCREMENT, task_id TEXT NOT NULL,
               dedup_key TEXT NOT NULL, loop_number INTEGER NOT NULL,
               route_point_id INTEGER NOT NULL DEFAULT 0,
               point_name TEXT NOT NULL DEFAULT '', direction TEXT NOT NULL DEFAULT '',
               segment_key TEXT NOT NULL DEFAULT '', category TEXT NOT NULL,
               confidence REAL NOT NULL, first_seen_at TEXT NOT NULL,
               last_seen_at TEXT NOT NULL, raw_image TEXT NOT NULL DEFAULT '',
               annotated_image TEXT NOT NULL DEFAULT '', map_x REAL, map_y REAL,
               acknowledged INTEGER NOT NULL DEFAULT 0, acknowledged_at TEXT,
               UNIQUE(task_id, dedup_key));''')
    connection.close()

    database = InspectionDatabase(str(db_path))
    with database._connection() as migrated:
        point_columns = {
            row['name'] for row in migrated.execute(
                'PRAGMA table_info(inspection_point_results)')}
        alert_columns = {
            row['name'] for row in migrated.execute(
                'PRAGMA table_info(inspection_alerts)')}
    assert {
        'thermal_raw_image', 'thermal_annotated_image',
        'thermal_measurement_json',
    } <= point_columns
    assert {
        'measurement_type', 'severity', 'measurement_json',
        'thermal_raw_image', 'thermal_annotated_image',
    } <= alert_columns


def test_report_numbers_stay_unique_for_matching_task_prefixes(tmp_path):
    database = InspectionDatabase(str(tmp_path / 'inspection.db'))
    template = database.save_template(7, {'checkpoints': []})
    task_ids = ('same-prefix-task-one', 'same-prefix-task-two')
    for task_id in task_ids:
        database.create_task(
            task_id, task_id,
            {'id': 7, 'name': 'route', 'map_name': 'map'},
            {'loop_count': 1}, template, _task_state(task_id),
        )
        database.update_task(
            task_id, {**_task_state(task_id), 'state': 'completed'},
            'completed',
        )

    reports = database.query_reports()['reports']
    report_numbers = {item['report_number'] for item in reports}
    assert len(reports) == 2
    assert len(report_numbers) == 2
