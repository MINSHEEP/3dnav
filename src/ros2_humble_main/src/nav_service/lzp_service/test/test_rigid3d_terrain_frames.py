"""Map prior must compare live and saved ground in the same 3D frame."""
from types import SimpleNamespace as NS
import numpy as np

from test_terrain_observer_runtime import harness
from lzp_service.terrain_grid import build_terrain_grid
from lzp_service.terrain_observer import estimate_local_terrain
from lzp_service.terrain_navigation import TerrainCapabilityModel
from lzp_service.terrain_pose import rigid_transform_matrix


def test_missing_one_return_cell_is_not_a_measured_trench_in_xingji3d():
    x, y = np.meshgrid(np.arange(.11, 1.29, .015), np.arange(-.44, .45, .02))
    points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    # A single missing 12cm sample band, with dense coplanar floor on both sides.
    points = points[(points[:, 0] < .58) | (points[:, 0] >= .70)]
    value = estimate_local_terrain(points, robot_x=0., robot_y=0., yaw=0.,
                                  separate_missing_returns=True)
    assert value.negative_obstacle is False
    assert value.gap_width_m == 0.
    assert value.sampling_gap_width_m > .1
    assert TerrainCapabilityModel().assess(value, check_confidence=False).traversable


def test_restricted_flat_prior_transforms_live_height_before_comparison():
    h = harness()
    x, y = np.meshgrid(np.arange(-3, 6, .05), np.arange(-3, 3, .05))
    measured = build_terrain_grid(np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .4))))
    grid = NS(**measured.__dict__)
    grid.map_name, grid.map_load_id = 'map', 'uuid'
    grid.header = NS(frame_id='map', stamp=NS(sec=1791830000, nanosec=123456789))
    h._grid, h._restricted_enabled = grid, True
    h._grid_status = dict(state='ready', artifact_verified=True,
        algorithm='clustered_measured_ground_v3', map_name='map', map_load_id='uuid',
        stamp_ns=1791830000123456789)
    h._map_pose = NS(header=NS(stamp=NS(sec=2, nanosec=0)), pose=NS(
        covariance=[0.]*36, pose=NS(position=NS(x=0., y=0., z=.4),
                                    orientation=NS(x=0., y=0., z=0., w=1.))))
    h._map_pose_samples = 2
    h._tf_buffer = NS(lookup_transform=lambda *_: NS(transform=NS(
        translation=NS(x=0., y=0., z=.4), rotation=NS(x=0., y=0., z=0., w=1.))))
    value = h._restricted_prior(h._points, 2., 0., 0., 0., ground_anchor=(0., 0., 0.))
    assert value is not None
    assert value.terrain_type == 'flat'


def test_terrain_frame_rotation_agrees_with_localization_rigid_geometry():
    from car_vel.rigid_pose import Pose3, quaternion, compose3
    pose = Pose3(1., -2., .434135, -.025442109, -.023617731, .3)
    matrix = np.asarray(rigid_transform_matrix((pose.x, pose.y, pose.z), quaternion(pose)))
    point = np.asarray((2., 1., -.52))
    expected = compose3(pose, Pose3(*point, 0., 0., 0.))
    assert np.allclose(matrix[:, :3] @ point + matrix[:, 3],
                       (expected.x, expected.y, expected.z), atol=1e-10)


def test_large_unobserved_gap_and_measured_drop_remain_blocked():
    x, y = np.meshgrid(np.arange(.11, 1.29, .015), np.arange(-.44, .45, .02))
    points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    points = points[(points[:, 0] < .46) | (points[:, 0] >= .94)]
    value = estimate_local_terrain(points, robot_x=0., robot_y=0., yaw=0.,
                                  separate_missing_returns=True)
    assert value.terrain_type == 'unknown'
    assert not TerrainCapabilityModel().assess(value, check_confidence=False).traversable
    from lzp_service.terrain_navigation import TerrainObservation
    assert not TerrainCapabilityModel().assess(
        TerrainObservation(negative_obstacle=True), check_confidence=False).traversable
