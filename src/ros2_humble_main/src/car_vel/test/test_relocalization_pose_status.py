import math

from geometry_msgs.msg import PoseWithCovarianceStamped

from car_vel.relocalization_coordinator import RelocalizationCoordinator


def test_pose_dictionary_reads_pose_orientation():
    message = PoseWithCovarianceStamped()
    message.pose.pose.position.x = -0.2
    message.pose.pose.position.y = -0.1
    expected_yaw = math.radians(3.0)
    message.pose.pose.orientation.z = math.sin(expected_yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(expected_yaw / 2.0)

    encoded = RelocalizationCoordinator._pose_dictionary(message)

    assert encoded['x'] == -0.2
    assert encoded['y'] == -0.1
    assert math.isclose(encoded['yaw'], expected_yaw, abs_tol=1.0e-5)
