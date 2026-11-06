#!/usr/bin/env python3
"""隔离运行默认 Fake 节点，验证 SIGINT 从正常线程完成关闭。"""

import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


STARTUP_TIMEOUT_SECONDS = 5
SHUTDOWN_TIMEOUT_SECONDS = 3
SERVICE_TIMEOUT_SECONDS = 3
HEARTBEAT_READY_SECONDS = 3
MIN_HEARTBEATS = 2


def confirm_executor_ready(probe, namespace):
    """真实服务响应证明 executor 已就绪，不能用构造日志代替。"""
    client = probe.create_client(
        GetState, f"{namespace}/astrall_driver/get_state"
    )
    try:
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SECONDS):
            raise RuntimeError("Private get_state service is unavailable")
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            probe, future, timeout_sec=SERVICE_TIMEOUT_SECONDS
        )
        if (not future.done() or
                future.result().current_state.id != State.PRIMARY_STATE_UNCONFIGURED):
            raise RuntimeError("Executor did not confirm unconfigured state")
    finally:
        probe.destroy_client(client)


def activate(probe, namespace):
    """只连接本测试子进程的私有生命周期服务，不修改生产启动策略。"""
    observed = {}

    def diagnostic(message):
        if message.status:
            observed.update({entry.key: entry.value
                             for entry in message.status[0].values})

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = probe.create_subscription(
        DiagnosticArray, f"{namespace}/driver_status", diagnostic, qos
    )
    client = probe.create_client(
        ChangeState, f"{namespace}/astrall_driver/change_state"
    )
    try:
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SECONDS):
            raise RuntimeError("Private lifecycle service is unavailable")
        for transition in (Transition.TRANSITION_CONFIGURE,
                           Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                probe, future, timeout_sec=SERVICE_TIMEOUT_SECONDS
            )
            if not future.done() or not future.result().success:
                raise RuntimeError(f"Lifecycle transition failed: {transition}")
        deadline = time.monotonic() + HEARTBEAT_READY_SECONDS
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
            if (observed.get("operational_active") == "1" and
                    int(observed.get("heartbeat_attempts", "0")) >= MIN_HEARTBEATS):
                return
        raise RuntimeError(f"Active heartbeat was not observed: {observed}")
    finally:
        probe.destroy_client(client)
        probe.destroy_subscription(subscription)


def run_case(executable, probe, case, active=False):
    namespace = f"/astrall_s2_exit_{os.getpid()}_{case}"
    process = subprocess.Popen(
        [str(executable), "--ros-args", "-r", f"__ns:={namespace}",
         "-p", "use_fake_sdk:=true"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    output = b""
    try:
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            while b"unconfigured" not in output:
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError(f"Node did not start: {output.decode()}")
                for key, _ in selector.select(timeout=0.1):
                    output += os.read(key.fd, 4096)
        if case != "startup":
            confirm_executor_ready(probe, namespace)
        if active:
            activate(probe, namespace)
        process.send_signal(signal.SIGINT)
        remaining, _ = process.communicate(timeout=SHUTDOWN_TIMEOUT_SECONDS)
        output += remaining
        expected = f"shutdown; zero attempted={int(active)}".encode()
        if process.returncode != 0 or expected not in output:
            raise RuntimeError(f"Exit={process.returncode}: {output.decode()}")
        print(f"PASS: isolated Fake main case={case}, active={active}, SIGINT exit=0, "
              f"zero_attempted={int(active)}")
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=SHUTDOWN_TIMEOUT_SECONDS)


def main():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Run with isolated CTest ROS_LOCALHOST_ONLY=1")
    executable = Path(sys.argv[1]).resolve(strict=True)
    rclpy.init(args=[])
    probe = rclpy.create_node(f"astrall_s2_exit_probe_{os.getpid()}")
    try:
        run_case(executable, probe, "startup")
        run_case(executable, probe, "unconfigured")
        run_case(executable, probe, "active", active=True)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
