"""Load all three CPU models in an owned worker; never open a camera or run ROS."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from .protocol import read_message


def check(timeout=90.0):
    root = Path(__file__).resolve().parents[1]
    env = dict(os.environ)
    for key in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD'):
        env.pop(key, None)
    env.update(PYTHONNOUSERSITE='1', CUDA_VISIBLE_DEVICES='',
               OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='2', MKL_NUM_THREADS='2')
    # Native model initialization logs stay out of the framed protocol stream.
    with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as errors:
        process = subprocess.Popen(
            [sys.executable, '-s', '-m', 'pphuman_runtime.worker', '--config', 'config/inference.yaml'],
            cwd=str(root), env=env, stdin=subprocess.DEVNULL, stdout=output, stderr=errors)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as error:
            process.kill()
            process.wait(timeout=5.0)
            raise RuntimeError('Behavior CPU model initialization timed out') from error
        output.seek(0)
        reply = read_message(output)
        if process.returncode or not reply or reply.get('type') != 'ready':
            errors.seek(0, os.SEEK_END)
            errors.seek(max(0, errors.tell() - 8192))
            raise RuntimeError('Behavior model load failed: ' + errors.read().decode('utf-8', 'replace'))
        if reply.get('runtime', {}).get('device') != 'CPU':
            raise RuntimeError('Worker did not confirm CPU inference')
        if set(reply.get('models', {})) != {'yolo_pose', 'fight', 'smoking'}:
            raise RuntimeError('Worker did not load all three expected behavior models')
        return {'success': True, 'device': 'CPU', 'models': list(reply.get('models', {})),
                'message': 'All three models loaded. Accuracy and real-time throughput are not measured.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=90.0)
    args = parser.parse_args()
    if not 1 <= args.timeout <= 300:
        parser.error('timeout must be between 1 and 300 seconds')
    print(json.dumps(check(args.timeout), ensure_ascii=False))


if __name__ == '__main__':
    main()
