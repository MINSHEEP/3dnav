"""Read-only /proc fixtures and actual guard orchestration without ROS/Linux."""
import signal
from unittest.mock import Mock, patch

from lzp_service import launch_process as guard


def process(proc, pid, *, group=4321, state='S', domain='0', token='ours',
            name='nav2_container'):
    directory = proc / str(pid)
    directory.mkdir()
    (directory / 'stat').write_text(f'{pid} (container (worker)) {state} 976 {group} 0')
    (directory / 'cmdline').write_bytes(
        f'/opt/ros/humble/lib/rclcpp_components/component_container_isolated\0'
        f'--ros-args\0-r\0__node:={name}\0'.encode())
    (directory / 'environ').write_bytes(
        f'ROS_DOMAIN_ID={domain}\0{guard.OWNER_TOKEN_ENV}={token}\0'.encode())
    return directory


def test_group_members_remain_visible_without_leader_and_ignore_zombies(tmp_path):
    process(tmp_path, 975382)
    process(tmp_path, 975383, state='Z')
    process(tmp_path, 980502, group=979378)
    assert guard.live_group_members(4321, proc_root=tmp_path) == [975382]
    assert guard.live_group_members(4321, exclude_pid=975382,
                                    proc_root=tmp_path) == []


def test_reused_group_number_does_not_authorize_killing_another_session(tmp_path):
    process(tmp_path, 975382, token='someone-else')
    assert guard.live_group_members(4321, owner_token='ours', proc_root=tmp_path) == []


def test_duplicate_containers_are_found_only_in_same_user_domain(tmp_path):
    first = process(tmp_path, 975382)
    process(tmp_path, 980502)
    process(tmp_path, 111, domain='42')
    process(tmp_path, 222, state='Z')
    process(tmp_path, 333, name='another_robot_container')
    result = guard.nav2_process_conflicts(
        proc_root=tmp_path, uid=first.stat().st_uid, environment={})
    assert [item['pid'] for item in result] == [975382, 980502]
    assert guard.nav2_process_conflicts(
        proc_root=tmp_path, uid=first.stat().st_uid + 1, environment={}) == []


def supervise_fixture(parent_sequence, members, *, timeout=False, closed_output=False):
    child = Mock(pid=43, returncode=0)
    child.poll.return_value = None if len(parent_sequence) > 1 else 0
    with patch.object(guard.os, 'getpid', return_value=42), \
            patch.object(guard.os, 'getpgrp', return_value=42, create=True), \
            patch.object(guard.os, 'getppid', side_effect=parent_sequence), \
            patch.object(guard.os, 'killpg', create=True) as killpg, \
            patch.object(guard.signal, 'signal'), \
            patch.object(guard.subprocess, 'Popen', return_value=child) as spawn, \
            patch.object(guard, 'live_group_members', side_effect=members), \
            patch.object(guard.time, 'sleep'), \
            patch.object(guard.time, 'monotonic', side_effect=[0, 99] if timeout else None,
                         return_value=0), \
            patch('builtins.print', side_effect=BrokenPipeError if closed_output else None):
        result = guard.supervise(['ros2', 'launch', 'car_vel', 'car_vel.launch.py'], 10)
        return result, killpg.call_args_list, spawn.call_args_list


def test_owner_disappearing_stops_its_group_even_if_terminal_has_closed():
    result, signals, spawns = supervise_fixture([10, 976], [[43], [43], []],
                                               closed_output=True)
    assert result == 0
    assert len(spawns) == 1
    assert [call.args for call in signals] == [(42, signal.SIGINT)]


def test_launch_leader_exit_still_cleans_surviving_container():
    _, signals, _ = supervise_fixture([10], [[44], [44], []])
    assert [call.args for call in signals] == [(42, signal.SIGINT)]


def test_guard_never_starts_after_owner_died():
    _, signals, spawns = supervise_fixture([976], [])
    assert not signals
    assert not spawns


def test_guard_escalates_only_its_dedicated_group_after_grace_period():
    _, signals, _ = supervise_fixture([10], [[44], [44]], timeout=True)
    assert [call.args for call in signals] == [
        (42, signal.SIGINT), (42, guard.KILL_SIGNAL)]
