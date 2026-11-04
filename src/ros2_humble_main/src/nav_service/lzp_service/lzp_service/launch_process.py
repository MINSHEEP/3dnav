"""Own a page-driven ROS launch group, including after its launch leader exits.

Linux process inspection is deliberately read-only. Unowned Nav2 processes are
reported as conflicts, never killed. The guard only signals its own new session.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

OWNER_TOKEN_ENV = 'ROBOT_MANAGED_LAUNCH_TOKEN'
KILL_SIGNAL = getattr(signal, 'SIGKILL', 9)


def _notice(message):
    try:
        print('NAV_LAUNCH_GUARD ' + message, flush=True)
    except OSError:
        pass  # Closing the old terminal must not prevent child cleanup.


def _stat(path):
    text = path.read_text()
    # comm may contain spaces and parentheses; fields follow the final ')'.
    fields = text[text.rfind(')') + 2:].split()
    return fields[0], int(fields[1]), int(fields[2])


def live_group_members(pgid, *, exclude_pid=None, owner_token=None, proc_root='/proc'):
    """Return non-zombie group members; a reaped group leader is not required."""
    result = []
    for directory in Path(proc_root).iterdir():
        if not directory.name.isdigit() or int(directory.name) == exclude_pid:
            continue
        relevant = False
        try:
            state, _, group = _stat(directory / 'stat')
            if group == pgid and state not in ('Z', 'X'):
                relevant = True
                if owner_token is not None:
                    expected = f'{OWNER_TOKEN_ENV}={owner_token}'.encode()
                    entries = (directory / 'environ').read_bytes().split(b'\0')
                    if expected not in entries:
                        continue
                result.append(int(directory.name))
        except PermissionError:
            if relevant and owner_token is not None:
                raise RuntimeError(f'无法确认进程组 {pgid} 的所属身份')
        except (OSError, ValueError, IndexError):
            continue  # /proc is a live view; process exit is normal.
    return result


def nav2_process_conflicts(*, proc_root='/proc', environment=None, uid=None):
    """Find same-user, same-domain Nav2 containers before opening a new tree."""
    root = Path(proc_root)
    if not root.is_dir():
        return []  # Non-Linux test/development hosts have no /proc.
    environment = os.environ if environment is None else environment
    domain = str(environment.get('ROS_DOMAIN_ID', '0') or '0')
    uid = os.getuid() if uid is None else uid
    conflicts = []
    for directory in root.iterdir():
        if not directory.name.isdigit():
            continue
        candidate = False
        try:
            if directory.stat().st_uid != uid:
                continue
            state, parent, group = _stat(directory / 'stat')
            if state in ('Z', 'X'):
                continue
            argv = [p.decode(errors='replace') for p in
                    (directory / 'cmdline').read_bytes().split(b'\0') if p]
            if not argv or Path(argv[0]).name not in (
                    'component_container', 'component_container_mt',
                    'component_container_isolated'):
                continue
            if not any(arg in ('__node:=nav2_container',
                               '__node:=/nav2_container') for arg in argv):
                continue
            candidate = True
            entries = (directory / 'environ').read_bytes().split(b'\0')
            remote = dict(entry.decode(errors='replace').split('=', 1)
                          for entry in entries if b'=' in entry)
            if str(remote.get('ROS_DOMAIN_ID', '0') or '0') != domain:
                continue
            conflicts.append(dict(pid=int(directory.name), ppid=parent,
                                  pgid=group, node='nav2_container'))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # Silently ignoring an unreadable same-user candidate would make
            # the preflight claim an absence it could not establish.
            if candidate:
                raise RuntimeError(f'无法检查导航进程 {directory.name} 的身份')
        except (ValueError, IndexError):
            continue
    return sorted(conflicts, key=lambda item: item['pid'])


def supervise(command, owner_pid, shutdown_timeout=8.0):
    """Run in a fresh session; clean children on owner death or launch exit."""
    group = os.getpid()
    if os.getpgrp() != group:
        raise RuntimeError('Launch guard requires a dedicated process group')
    stopping = []

    def request_stop(signum, _frame):
        stopping.append(signum)

    for signum in (signal.SIGINT, signal.SIGTERM, getattr(signal, 'SIGHUP', 1)):
        signal.signal(signum, request_stop)
    if os.getppid() != owner_pid:
        return 1  # Owner died before this guard finished starting.
    process = subprocess.Popen(command, stdin=subprocess.DEVNULL)
    _notice(f'started owner={owner_pid} group={group} launch_pid={process.pid}')
    reason = 'launch_exited'
    try:
        while process.poll() is None and not stopping:
            if os.getppid() != owner_pid:
                reason = 'owner_exited'
                break
            time.sleep(0.1)
        if stopping:
            reason = 'stop_requested'
    finally:
        _notice(f'cleanup reason={reason} group={group}')
        if live_group_members(group, exclude_pid=group):
            # Our own handler keeps the guard alive until its children exit.
            os.killpg(group, signal.SIGINT)
            deadline = time.monotonic() + shutdown_timeout
            while live_group_members(group, exclude_pid=group):
                if time.monotonic() >= deadline:
                    _notice(f'forcing owned group={group}')
                    os.killpg(group, KILL_SIGNAL)
                    break
                process.poll()  # Reap launch leader even while a child remains.
                time.sleep(0.05)
        process.poll()
    return process.returncode if process.returncode is not None else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--owner-pid', type=int, required=True)
    parser.add_argument('--shutdown-timeout', type=float, default=8.0)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('a launch command is required')
    return supervise(command, args.owner_pid, max(0.1, args.shutdown_timeout))


if __name__ == '__main__':
    raise SystemExit(main())
