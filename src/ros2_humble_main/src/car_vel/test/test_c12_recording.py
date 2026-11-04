"""Recording lifecycle runs child processes without touching a camera."""
import os
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'hikvision_camera'))
from c12_recording import C12Recorder


def wait_for(predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError('录像状态未及时更新')


@pytest.fixture
def recorder(tmp_path, monkeypatch):
    executable = tmp_path / 'ffmpeg'
    executable.write_text(f'#!{sys.executable}\nimport os, pathlib, signal, sys, time\n'
                          'def stop(*_):\n    time.sleep(float(os.environ.get("C12_TEST_STOP_DELAY", "0")))\n    sys.exit(0)\n'
                          'signal.signal(signal.SIGINT, stop)\n'
                          'pathlib.Path(sys.argv[-1]).write_bytes(b"x" * 5000)\n'
                          'while True: time.sleep(.1)\n')
    executable.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path) + os.pathsep + os.environ['PATH'])
    recorder = C12Recorder('rtsp://visible', 'rtsp://infrared', tmp_path / '录像')
    yield recorder
    recorder.close()


def test_two_files_start_stop_and_duplicate(recorder):
    state = recorder.start()
    assert state['state'] == 'starting'
    with pytest.raises(ValueError):
        recorder.start()
    wait_for(lambda: recorder.status()['state'] == 'recording')
    assert all(Path(path).stat().st_size == 5000 for path in recorder.status()['files'].values())
    assert recorder.stop()['state'] == 'stopping'
    wait_for(lambda: recorder.status()['state'] == 'idle')
    assert not recorder.processes


def test_one_stream_failure_stops_other(recorder):
    recorder.start()
    wait_for(lambda: recorder.status()['state'] == 'recording')
    other = recorder.processes['infrared']
    recorder.processes['visible'].terminate()
    wait_for(lambda: recorder.status()['state'] == 'error')
    assert 'visible' in recorder.status()['error']
    assert other.poll() is not None


def test_recording_http_api(recorder, monkeypatch):
    monkeypatch.setenv('C12_TEST_STOP_DELAY', '2')
    from http.server import ThreadingHTTPServer
    from types import SimpleNamespace
    from urllib.request import Request, urlopen
    import json
    import threading
    import camera_web

    server = ThreadingHTTPServer(('127.0.0.1', 0), camera_web.CameraRequestHandler)
    server.gateway = SimpleNamespace(c12_recorder=recorder,
        camera_source_status=lambda: {'config': {'pipeline': 'c12'}})
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    url = f'http://127.0.0.1:{server.server_port}/api/c12/recording'
    def post(operation):
        request = Request(url, data=json.dumps({'operation': operation}).encode(),
                          headers={'Content-Type': 'application/json'})
        with urlopen(request, timeout=1) as response:
            return json.load(response)
    try:
        assert post('start')['state'] == 'starting'
        wait_for(lambda: recorder.status()['state'] == 'recording')
        with urlopen(url, timeout=5) as response:
            assert json.load(response)['state'] == 'recording'
        assert post('stop')['state'] == 'stopping'
        with urlopen(url, timeout=1) as response:
            assert json.load(response)['state'] == 'stopping'
        wait_for(lambda: recorder.status()['state'] == 'idle')
    finally:
        server.shutdown()
        server.server_close()
        worker.join()


def test_slow_encoder_shutdown_does_not_block_stop_or_status(recorder, monkeypatch):
    monkeypatch.setenv('C12_TEST_STOP_DELAY', '2')
    recorder.start()
    wait_for(lambda: recorder.status()['state'] == 'recording')
    started = time.monotonic()
    assert recorder.stop()['state'] == 'stopping'
    assert recorder.status()['state'] == 'stopping'
    assert time.monotonic() - started < 0.5
    with pytest.raises(ValueError):
        recorder.start()
    wait_for(lambda: recorder.status()['state'] == 'idle')
    assert not recorder.processes


def test_stop_during_start_and_restart_uses_new_files(recorder):
    first = recorder.start()['files']
    recorder.stop()
    wait_for(lambda: recorder.status()['state'] == 'idle')
    second = recorder.start()['files']
    assert first != second
    wait_for(lambda: recorder.status()['state'] == 'recording')
    children = list(recorder.processes.values())
    recorder.close()
    assert recorder.status()['state'] == 'idle'
    assert not recorder.processes
    assert all(child.poll() is not None for child in children)
