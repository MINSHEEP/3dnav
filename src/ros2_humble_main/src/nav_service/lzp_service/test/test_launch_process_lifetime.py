"""A ros2 launch parent exiting must not hide its live component container."""
from unittest.mock import Mock

import test_launch_manager_fastdds_preflight as _fixtures


def manager_with_orphan():
    fixture = _fixtures.TestFastDdsPreflight()
    fixture.setUp()
    manager = fixture.manager
    manager.shutdown_timeout_sec = 0.01
    manager.force_kill_timeout_sec = 0.01
    process = Mock(pid=4321, returncode=0)
    process.poll.return_value = 0
    process.wait.return_value = 0
    manager.running_processes['car_vel'] = process
    manager._process_tree_alive = Mock(return_value=True)
    manager._signal_process_tree = Mock()
    return manager, process


def test_exited_launch_parent_does_not_discard_live_owned_children():
    manager, process = manager_with_orphan()
    assert manager._get_managed_process('car_vel') is process
    manager._reap_exited_processes()
    assert manager.running_processes['car_vel'] is process


def test_stop_cannot_report_success_while_owned_child_survives():
    manager, process = manager_with_orphan()
    success, message = manager._stop_mode_locked('car_vel')
    assert not success, message
    assert manager.running_processes['car_vel'] is process
    assert manager._signal_process_tree.call_count >= 2


def test_signal_uses_owned_group_id_even_after_leader_is_reaped():
    fixture = _fixtures.TestFastDdsPreflight()
    fixture.setUp()
    from unittest.mock import patch
    process = Mock(pid=4321)
    process._robot_owner_token = None
    with patch.object(fixture.module.os, 'name', 'posix'), \
            patch.object(fixture.module.os, 'getpgid', create=True,
                         side_effect=ProcessLookupError) as lookup, \
            patch.object(fixture.module.os, 'killpg', create=True) as killpg:
        fixture.manager._signal_process_tree(process, 15)
    lookup.assert_not_called()
    killpg.assert_called_once_with(4321, 15)


def test_foreign_container_blocks_start_before_spawning_another_tree():
    from unittest.mock import patch
    fixture = _fixtures.TestFastDdsPreflight()
    fixture.setUp()
    with patch.object(fixture.module, 'nav2_process_conflicts', return_value=[
            dict(pid=975382, ppid=976, pgid=975000)]), \
            patch.object(fixture.module.subprocess, 'Popen') as spawn:
        success, message = fixture.manager._start_mode('car_vel')
    assert not success
    assert '975382' in message
    spawn.assert_not_called()


def test_stop_removes_record_only_after_whole_group_exits():
    manager, process = manager_with_orphan()
    manager._process_tree_alive.side_effect = [True, False, False]
    success, message = manager._stop_mode_locked('car_vel')
    assert success, message
    assert 'car_vel' not in manager.running_processes
    manager._signal_process_tree.assert_called_once()
