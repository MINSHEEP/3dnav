#!/usr/bin/env python3
"""Forward web ``/web_goal_pose`` messages to Nav2's NavigateToPose action."""

import copy
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty, String


class GoalPoseBridge(Node):
    """Keep the browser API topic-based while using Nav2's action API."""

    DUPLICATE_GOAL_WINDOW_SEC = 0.75
    DUPLICATE_GOAL_POSITION_TOLERANCE = 0.01
    DUPLICATE_GOAL_YAW_TOLERANCE = math.radians(1.0)

    def __init__(self):
        super().__init__('goal_pose_bridge')

        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            # rosbridge currently offers TRANSIENT_LOCAL.  A VOLATILE
            # subscription is compatible and avoids replaying an old goal
            # whenever navigation mode is restarted.
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        # Nav2 Humble's bt_navigator already subscribes to /goal_pose and
        # forwards that topic to its own NavigateToPose Action client.  Using
        # the same topic here would execute every browser goal twice and cause
        # an immediate preemption.  Keep the browser ingress private so this
        # bridge is the sole Action client for web goals.
        self._goal_subscription = self.create_subscription(
            PoseStamped, '/web_goal_pose', self._goal_callback, goal_qos)
        self._cancel_subscription = self.create_subscription(
            Empty, '/cancel_navigation', self._cancel_callback, goal_qos)
        self._status_publisher = self.create_publisher(
            String, '/goal_pose_status', status_qos)
        self._readiness_publisher = self.create_publisher(
            Bool, '/nav2_ready', status_qos)
        self._active_publisher = self.create_publisher(
            Bool, '/navigation_active', status_qos)

        self._pending_pose = None
        self._sending_pose = None
        self._active_pose = None
        self._active_goal_handle = None
        self._send_in_progress = False
        self._send_generation = 0
        self._send_started_at = None
        self._cancel_in_progress = False
        self._cancel_started_at = None
        self._cancel_attempts = 0
        self._cancel_generation = 0
        self._cancel_requested = False
        self._last_received_goal = None
        self._last_received_goal_at = None
        self._nav2_ready = False
        self._nav2_wait_started = time.monotonic()
        self._last_status = None
        self._dispatch_timer = self.create_timer(0.5, self._timer_callback)

        self._publish_status('ready')
        self.get_logger().info(
            'Goal pose bridge is ready: /web_goal_pose -> /navigate_to_pose')

    def _timer_callback(self):
        now = time.monotonic()
        if (
            self._send_in_progress
            and self._send_started_at is not None
            and now - self._send_started_at > 5.0
        ):
            # Invalidate the outstanding callback.  If it later returns an
            # accepted goal, that callback cancels it instead of activating it.
            self._send_generation += 1
            self._send_in_progress = False
            self._send_started_at = None
            self._sending_pose = None
            self._cancel_requested = False
            self.get_logger().error('Timed out while sending the Nav2 goal.')
            self._publish_status('send_failed')

        if (
            self._cancel_in_progress
            and self._cancel_started_at is not None
            and now - self._cancel_started_at > 3.0
        ):
            self._cancel_in_progress = False
            self._cancel_started_at = None
            if self._cancel_attempts < 3 and self._active_goal_handle is not None:
                self.get_logger().warn('Navigation cancel timed out; retrying.')
                self._cancel_active_goal()
            else:
                # Ignore a response that arrives after the final timeout.
                self._cancel_generation += 1
                # A timeout does not prove whether Nav2 accepted the cancel.
                # Keep the pending replacement until the action result tells
                # us whether the old goal succeeded or was canceled/aborted.
                self.get_logger().error(
                    'Unable to confirm navigation cancellation.')
                self._publish_status('cancel_failed')

        nav2_ready = self._action_client.server_is_ready()
        self._readiness_publisher.publish(Bool(data=nav2_ready))
        self._active_publisher.publish(Bool(data=bool(
            self._pending_pose is not None
            or self._active_goal_handle is not None
            or self._send_in_progress
            or self._cancel_in_progress)))
        if nav2_ready and not self._nav2_ready:
            self._nav2_ready = True
            self._nav2_wait_started = time.monotonic()
            self._publish_status('nav2_ready')
            self.get_logger().info('Nav2 NavigateToPose action server is ready.')
        elif not nav2_ready and self._nav2_ready:
            self._nav2_ready = False
            self._nav2_wait_started = time.monotonic()
            self._publish_status('waiting_for_nav2')
        elif (
            not nav2_ready
            and time.monotonic() - self._nav2_wait_started > 30.0
            and self._last_status in ('ready', 'waiting_for_nav2')
        ):
            self._publish_status('nav2_start_timeout')

        self._dispatch_pending_goal()

    def _publish_status(self, status):
        if status == self._last_status:
            return
        self._last_status = status
        self._status_publisher.publish(String(data=status))

    @staticmethod
    def _goal_yaw(pose):
        quaternion = pose.pose.orientation
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(sin_yaw, cos_yaw)

    @classmethod
    def _same_navigation_goal(cls, first, second):
        if first.header.frame_id != second.header.frame_id:
            return False
        position_distance = math.hypot(
            first.pose.position.x - second.pose.position.x,
            first.pose.position.y - second.pose.position.y,
        )
        yaw_delta = cls._goal_yaw(first) - cls._goal_yaw(second)
        yaw_distance = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
        return (
            position_distance <= cls.DUPLICATE_GOAL_POSITION_TOLERANCE
            and yaw_distance <= cls.DUPLICATE_GOAL_YAW_TOLERANCE
        )

    def _coalesce_pending_active_goal(self):
        """Keep the active goal when it already is the newest requested pose."""
        if (
            self._pending_pose is None
            or self._active_pose is None
            or not self._same_navigation_goal(
                self._pending_pose, self._active_pose)
        ):
            return False
        self._pending_pose = None
        self._cancel_requested = False
        self._cancel_attempts = 0
        self._publish_status('navigating')
        return True

    def _goal_callback(self, pose):
        if not pose.header.frame_id:
            pose.header.frame_id = 'map'

        received_at = time.monotonic()
        # Compare with the newest requested destination, not every historical
        # in-flight pose.  This preserves a legitimate A -> B -> A change while
        # still making repeated delivery of the current request idempotent.
        expected_pose = self._pending_pose
        if expected_pose is None:
            expected_pose = self._sending_pose
        if (
            expected_pose is None
            and not self._cancel_in_progress
            and not self._cancel_requested
        ):
            expected_pose = self._active_pose
        equivalent_in_flight = (
            expected_pose is not None
            and self._same_navigation_goal(expected_pose, pose)
        )
        if (
            equivalent_in_flight
            or (
                self._last_received_goal is not None
                and self._last_received_goal_at is not None
                and received_at - self._last_received_goal_at
                <= self.DUPLICATE_GOAL_WINDOW_SEC
                and self._same_navigation_goal(self._last_received_goal, pose)
            )
        ):
            self.get_logger().warn(
                'Ignored a navigation goal that is already pending or active.')
            return
        self._last_received_goal = copy.deepcopy(pose)
        self._last_received_goal_at = received_at

        # Keep only the newest browser request.  This also prevents a stale
        # transient-local rosbridge sample from being executed after restart.
        self._cancel_requested = False
        self._pending_pose = copy.deepcopy(pose)
        self._publish_status('goal_queued')

        if self._active_goal_handle is not None:
            if not self._cancel_in_progress:
                self._cancel_attempts = 0
            self._cancel_active_goal()
        else:
            self._dispatch_pending_goal()

    def _cancel_callback(self, _message):
        """Cancel autonomous motion before teleoperation or an emergency stop."""
        self._pending_pose = None
        self._cancel_requested = True
        self._publish_status('canceling_goal')

        if self._active_goal_handle is not None:
            if not self._cancel_in_progress:
                self._cancel_attempts = 0
            self._cancel_active_goal()
        elif not self._send_in_progress:
            self._cancel_requested = False
            self._publish_status('goal_canceled')

    def _dispatch_pending_goal(self):
        if (
            self._pending_pose is None
            or self._active_goal_handle is not None
            or self._send_in_progress
            or self._cancel_in_progress
            or self._cancel_requested
        ):
            return

        if not self._nav2_ready and not self._action_client.server_is_ready():
            self._publish_status('waiting_for_nav2')
            return

        pose = self._pending_pose
        self._pending_pose = None
        self._sending_pose = copy.deepcopy(pose)

        # A zero stamp means "latest" on the browser topic.  Replace it with
        # the robot clock before passing the goal to Nav2.
        if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
            pose.header.stamp = self.get_clock().now().to_msg()

        action_goal = NavigateToPose.Goal()
        action_goal.pose = pose

        self._send_in_progress = True
        self._send_started_at = time.monotonic()
        self._send_generation += 1
        generation = self._send_generation
        self._publish_status('sending_goal')
        future = self._action_client.send_goal_async(action_goal)
        future.add_done_callback(
            lambda result, token=generation:
            self._goal_response_callback(result, token))

    def _goal_response_callback(self, future, generation):
        if generation != self._send_generation:
            try:
                late_goal_handle = future.result()
                if late_goal_handle.accepted:
                    late_goal_handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().warn(
                    f'Late navigation goal response failed: {error}')
            return

        self._send_in_progress = False
        self._send_started_at = None
        sent_pose = self._sending_pose
        self._sending_pose = None
        try:
            goal_handle = future.result()
        except Exception as error:  # Action transport/runtime failure.
            self._cancel_requested = False
            self.get_logger().error(f'Failed to send navigation goal: {error}')
            self._publish_status('send_failed')
            self._dispatch_pending_goal()
            return

        if not goal_handle.accepted:
            self._cancel_requested = False
            self.get_logger().warn('Nav2 rejected the navigation goal.')
            self._publish_status('goal_rejected')
            self._dispatch_pending_goal()
            return

        self._active_goal_handle = goal_handle
        self._active_pose = sent_pose
        self._cancel_attempts = 0
        self._publish_status('navigating')
        self.get_logger().info('Nav2 accepted the navigation goal.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, handle=goal_handle: self._result_callback(result, handle))

        # A -> B -> A may occur while A is still being accepted.  In that case
        # the accepted goal already equals the newest request, so no
        # cancel/re-send cycle is needed.
        self._coalesce_pending_active_goal()

        # A newer goal may have arrived while the action request was being
        # accepted.  Cancel this one before dispatching the latest request.
        if self._cancel_requested or self._pending_pose is not None:
            self._cancel_active_goal()

    def _cancel_active_goal(self):
        if self._active_goal_handle is None or self._cancel_in_progress:
            return
        self._cancel_in_progress = True
        self._cancel_started_at = time.monotonic()
        self._cancel_attempts += 1
        self._cancel_generation += 1
        generation = self._cancel_generation
        self._publish_status(
            'canceling_goal'
            if self._cancel_requested and self._pending_pose is None
            else 'replacing_goal')
        future = self._active_goal_handle.cancel_goal_async()
        future.add_done_callback(
            lambda result, token=generation:
            self._cancel_response_callback(result, token))

    def _cancel_response_callback(self, future, generation):
        if generation != self._cancel_generation:
            return
        self._cancel_in_progress = False
        self._cancel_started_at = None
        cancellation_accepted = False
        try:
            response = future.result()
            cancellation_accepted = bool(response.goals_canceling)
            if not cancellation_accepted:
                self.get_logger().warn(
                    'Nav2 did not accept cancellation of the previous goal.')
        except Exception as error:
            self.get_logger().warn(f'Unable to cancel the previous goal: {error}')

        if not cancellation_accepted and self._active_goal_handle is not None:
            # If A -> B -> A was requested while cancellation of A was already
            # in flight, a rejected cancellation means A is still exactly the
            # newest desired goal.  Coalesce it instead of running A again when
            # the current action eventually finishes.
            if self._coalesce_pending_active_goal():
                return
            self._publish_status('cancel_failed')
            return

        # The result callback clears the active handle.  If Nav2 has already
        # completed it, dispatching here is harmless and reduces latency.
        if self._active_goal_handle is None:
            self._dispatch_pending_goal()

    def _result_callback(self, future, goal_handle):
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:
            self.get_logger().error(f'Unable to read navigation result: {error}')
            status = GoalStatus.STATUS_UNKNOWN

        if goal_handle is self._active_goal_handle:
            completed_pose = self._active_pose
            if (
                status == GoalStatus.STATUS_SUCCEEDED
                and self._pending_pose is not None
                and completed_pose is not None
                and self._same_navigation_goal(
                    self._pending_pose, completed_pose)
            ):
                # A -> B -> A can race with A completing before its cancel
                # response.  A is already reached, so do not execute it twice.
                self._pending_pose = None
            self._active_goal_handle = None
            self._active_pose = None
            self._cancel_generation += 1
            self._cancel_in_progress = False
            self._cancel_started_at = None
            self._cancel_attempts = 0
            self._cancel_requested = False

        status_names = {
            GoalStatus.STATUS_SUCCEEDED: 'goal_succeeded',
            GoalStatus.STATUS_ABORTED: 'goal_aborted',
            GoalStatus.STATUS_CANCELED: 'goal_canceled',
        }
        self._publish_status(status_names.get(status, 'goal_failed'))
        self.get_logger().info(
            f'Navigation goal finished with status {status_names.get(status, status)}.')
        self._dispatch_pending_goal()

    def cancel_before_shutdown(self, timeout_sec=1.5):
        """Best-effort cancellation if this bridge is stopped on its own."""
        self._pending_pose = None
        if self._active_goal_handle is None:
            return
        try:
            future = self._active_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        except Exception as error:
            self.get_logger().warn(
                f'Unable to cancel navigation during shutdown: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_before_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
