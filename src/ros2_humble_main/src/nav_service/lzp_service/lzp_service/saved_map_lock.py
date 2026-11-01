"""OS-released cross-process edit/save lease; never persists a stale lock."""
from contextlib import contextmanager
import os
from pathlib import Path


def _set_lock(stream, acquire):
    stream.seek(0)
    if os.name == 'nt':
        import msvcrt
        msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK if acquire else msvcrt.LK_UNLCK, 1)
    else:
        import fcntl
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB if acquire else fcntl.LOCK_UN)


@contextmanager
def saved_map_lock(directory):
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True)
    with (root/'.saved-map-write.lock').open('a+b') as stream:
        if stream.tell() == 0:
            stream.write(b'0')
            stream.flush()
        try:
            _set_lock(stream, True)
        except OSError as error:
            raise ValueError('另一个地图编辑/保存正在进行，请稍后重试') from error
        try:
            yield
        finally:
            _set_lock(stream, False)


def saved_map_write_busy(directory):
    if not Path(directory).exists():
        return False
    try:
        with saved_map_lock(directory):
            return False
    except (OSError, ValueError):
        return True
