"""Process-scoped hardware ownership; also protects legacy command entrypoints."""
from pathlib import Path


def acquire_body_lock(expected_body=None):
    import fcntl
    folder = Path.home() / '.ros' / 'robot_navigation'
    folder.mkdir(parents=True, exist_ok=True)
    if expected_body is not None:
        from lzp_service.robot_body import BodyConfigStore
        if BodyConfigStore().load()['selected'] != expected_body:
            raise RuntimeError('所选本体不是星玑；禁止旧 UDP 入口控制其他本体')
    stream = (folder / 'body_control.lock').open('a')
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        stream.close()
        raise RuntimeError('已有本体控制进程运行；拒绝启动第二个 SDK/UDP 控制者')
    return stream
