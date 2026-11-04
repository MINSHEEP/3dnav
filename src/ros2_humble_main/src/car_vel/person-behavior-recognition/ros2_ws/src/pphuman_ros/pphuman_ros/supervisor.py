"""Foreground owner of this invocation's children; safe identity-checked stop."""
import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from .common import node_lease, project_root


ROLE_NAMES = {'camera': '相机', 'recognition': '识别', 'view': '预览'}


@dataclass
class Child:
    role: str
    process: object
    required: bool = True
    exit_reported: bool = False


def report(message):
    print('[supervisor] ' + message, file=sys.stderr, flush=True)


def poll_children(children, logger=report):
    """Optional preview exit is visible to the user and never stops core nodes."""
    for child in children:
        result = child.process.poll()
        if result is None or child.exit_reported:
            continue
        child.exit_reported = True
        identity = f'{ROLE_NAMES[child.role]}({child.role}) PID={child.process.pid}'
        if child.required:
            logger(f'{identity} 意外退出，退出码={result}；停止本次其余自有进程。')
            return result if result > 0 else 1
        reason = '已关闭' if result == 0 else '异常退出'
        logger(f'{identity} {reason}，退出码={result}；相机和识别继续运行。'
               '可用 bash scripts/view_results.sh 重新打开预览；全部停止请 Ctrl+C 或 bash scripts/stop_all.sh。')
    return None


def recognition_command(root, image_topic, processing_fps=None, recognition_config=None):
    command = ['bash', str(root / 'scripts/start_recognition.sh'), '--ros-args']
    if recognition_config:
        command += ['--params-file', str(Path(recognition_config).resolve())]
    command += ['-p', f'image_topic:={image_topic}']
    if processing_fps is not None:
        command += ['-p', f'processing_fps:={processing_fps}']
    return command


def start_ticks(pid):
    # /proc stat command may contain spaces or parentheses; split after final ')'.
    return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]


def registry_path():
    return project_root() / 'run' / ('supervisor-' + os.environ.get('ROS_DOMAIN_ID', '0') + '.json')


def stop_running():
    path = registry_path()
    if not path.exists():
        print('没有本项目 start_all 记录的运行实例。独立前台节点请 Ctrl+C 停止。')
        return
    record = json.loads(path.read_text())
    pid = int(record['pid'])
    try:
        cmdline = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        if record['root'] != str(project_root()) or start_ticks(pid) != record['start_ticks'] or b'pphuman_ros.supervisor' not in cmdline:
            raise RuntimeError('进程身份不匹配，拒绝发送信号')
        os.kill(pid, signal.SIGTERM)
        print(f'已请求本项目监督进程 {pid} 停止并回收其子进程。')
    except FileNotFoundError:
        print('记录对应进程已经退出。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--no-camera', action='store_true', help='只订阅现有 image_topic')
    parser.add_argument('--no-view', action='store_true', help='无桌面运行')
    parser.add_argument('--image-topic', default='/pp_human/image_raw')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--processing-fps', type=float, default=None,
                        help='仅显式指定时覆盖 ROS 参数文件中的 processing_fps')
    parser.add_argument('--recognition-config', help='附加 ROS 参数文件，覆盖 config/ros.yaml')
    args = parser.parse_args()
    if args.stop:
        stop_running()
        return
    lease = node_lease('supervisor')
    root = project_root()
    path = registry_path()
    path.parent.mkdir(exist_ok=True)
    children = []
    stopping = False

    def on_signal(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    code = 0
    try:
        path.write_text(json.dumps(dict(pid=os.getpid(), start_ticks=start_ticks(os.getpid()), root=str(root))))
        commands = []
        if not args.no_camera:
            commands.append(('camera', ['bash', str(root / 'scripts/start_camera.sh'), '--ros-args',
                '-p', f'device:={args.device}', '-p', f'image_topic:={args.image_topic}',
                '-p', f'width:={args.width}', '-p', f'height:={args.height}', '-p', f'fps:={args.fps}']))
        commands.append(('recognition', recognition_command(root, args.image_topic,
                         args.processing_fps, args.recognition_config)))
        if not args.no_view:
            commands.append(('view', ['bash', str(root / 'scripts/view_results.sh')]))
        for role, command in commands:
            process = subprocess.Popen(command, cwd=root, start_new_session=True)
            children.append(Child(role, process, required=role != 'view'))
            report(f'已启动 {ROLE_NAMES[role]}({role}) PID={process.pid}')
        while not stopping:
            core_exit = poll_children(children)
            if core_exit is not None:
                code = core_exit
                break
            time.sleep(.2)
    finally:
        for owned in children:
            child = owned.process
            if child.poll() is None:
                report(f'停止自有 {ROLE_NAMES[owned.role]}({owned.role}) PID={child.pid}')
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 12
        for owned in children:
            child = owned.process
            try:
                child.wait(timeout=max(.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=3)
        path.unlink(missing_ok=True)
        lease.close()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
