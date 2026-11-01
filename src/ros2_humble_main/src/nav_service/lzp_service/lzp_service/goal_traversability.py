"""Pure target-cost and connected-component validation for Nav2 goals."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GoalValidationResult:
    """Decision and operator-facing evidence for one navigation target."""

    allowed: bool
    code: str
    message: str
    nearest_start_component_distance_m: float = math.inf
    nearest_start_traversable_distance_m: float = math.inf


@dataclass(frozen=True)
class TraversabilityGrid:
    """A validated cost grid with cached 8-connected component labels."""

    data: np.ndarray
    labels: np.ndarray
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    @classmethod
    def from_costs(
            cls, *, data, resolution, width, height,
            origin_x, origin_y, origin_yaw=0.0):
        values = (
            resolution, origin_x, origin_y, origin_yaw,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('traversability geometry must be finite')
        width = int(width)
        height = int(height)
        resolution = float(resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise ValueError('traversability geometry must be positive')
        costs = np.asarray(data, dtype=np.int16).reshape(-1)
        if costs.size != width * height:
            raise ValueError('traversability grid length is invalid')
        if np.any((costs < -1) | (costs > 100)):
            raise ValueError('traversability costs must be in [-1, 100]')
        matrix = costs.reshape(height, width)
        labels = _component_labels((matrix >= 0) & (matrix < 100))
        return cls(
            data=matrix.astype(np.int8),
            labels=labels,
            resolution=resolution,
            width=width,
            height=height,
            origin_x=float(origin_x),
            origin_y=float(origin_y),
            origin_yaw=float(origin_yaw),
        )

    def world_to_cell(self, x, y):
        delta_x = float(x) - self.origin_x
        delta_y = float(y) - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        column = int(math.floor(local_x / self.resolution))
        row = int(math.floor(local_y / self.resolution))
        if not (0 <= row < self.height and 0 <= column < self.width):
            return None
        return row, column

    def cell_centre(self, row, column):
        local_x = (float(column) + 0.5) * self.resolution
        local_y = (float(row) + 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )


def _component_labels(traversable):
    labels = np.zeros(traversable.shape, dtype=np.int32)
    component = 0
    height, width = traversable.shape
    for row, column in zip(*np.nonzero(traversable)):
        if labels[row, column] != 0:
            continue
        component += 1
        labels[row, column] = component
        stack = [(int(row), int(column))]
        while stack:
            current_row, current_column = stack.pop()
            for delta_row in (-1, 0, 1):
                for delta_column in (-1, 0, 1):
                    if delta_row == 0 and delta_column == 0:
                        continue
                    next_row = current_row + delta_row
                    next_column = current_column + delta_column
                    if not (
                        0 <= next_row < height
                        and 0 <= next_column < width
                    ):
                        continue
                    if (
                        traversable[next_row, next_column]
                        and labels[next_row, next_column] == 0
                    ):
                        labels[next_row, next_column] = component
                        stack.append((next_row, next_column))
    return labels


def validate_navigation_goal(
        grid, *, start_x, start_y, goal_x, goal_y,
        goal_tolerance_m=0.10, start_tolerance_m=0.35):
    """Require the goal or its tolerated substitute to share start's island."""

    values = (
        start_x, start_y, goal_x, goal_y,
        goal_tolerance_m, start_tolerance_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return GoalValidationResult(
            False, 'invalid_coordinates', '目标或机器人坐标无效')
    start = grid.world_to_cell(start_x, start_y)
    goal = grid.world_to_cell(goal_x, goal_y)
    if start is None:
        return GoalValidationResult(
            False, 'robot_outside_map', '机器人当前位置不在可导航地图范围内')
    if goal is None:
        return GoalValidationResult(
            False, 'goal_outside_map', '目标点超出当前地图范围')
    start_label = int(grid.labels[start])
    start_snap_distance = math.inf
    if start_label <= 0:
        # A reconstructed 0.25 m terrain cell can remain unknown because the
        # lidar has a near-field/self-occlusion hole even on a flat floor.
        # Resolve only UNKNOWN starts to a tightly bounded, already verified
        # component.  Lethal cells, map-outside coordinates and larger gaps
        # retain veto power.
        start_cost = int(grid.data[start])
        radius_cells = max(
            0, int(math.ceil(float(start_tolerance_m) / grid.resolution)))
        start_row, start_column = start
        row_start = max(0, start_row - radius_cells)
        row_end = min(grid.height, start_row + radius_cells + 1)
        column_start = max(0, start_column - radius_cells)
        column_end = min(grid.width, start_column + radius_cells + 1)
        local_labels = grid.labels[
            row_start:row_end, column_start:column_end]
        local_rows, local_columns = np.nonzero(local_labels > 0)
        if start_cost == -1 and local_rows.size:
            traversable_rows = local_rows + row_start
            traversable_columns = local_columns + column_start
            centres = np.asarray([
                grid.cell_centre(row, column)
                for row, column in zip(
                    traversable_rows, traversable_columns)
            ], dtype=np.float64)
            distances = np.hypot(
                centres[:, 0] - float(start_x),
                centres[:, 1] - float(start_y),
            )
            nearest_index = int(np.argmin(distances))
            start_snap_distance = float(distances[nearest_index])
            if start_snap_distance <= float(start_tolerance_m):
                start_label = int(grid.labels[
                    traversable_rows[nearest_index],
                    traversable_columns[nearest_index],
                ])
        if start_label <= 0:
            if start_cost >= 100:
                detail = (
                    f'机器人位置({float(start_x):.2f}, '
                    f'{float(start_y):.2f})落在禁止通行格，'
                    '请重新定位或检查地形')
            else:
                detail = (
                    f'机器人位置({float(start_x):.2f}, '
                    f'{float(start_y):.2f})位于地形未知区，'
                    f'{float(start_tolerance_m):.2f} m内没有已验证可通行格')
            return GoalValidationResult(
                False,
                'robot_not_traversable',
                detail,
                nearest_start_traversable_distance_m=start_snap_distance,
            )
    goal_label = int(grid.labels[goal])
    if goal_label == start_label:
        return GoalValidationResult(
            True,
            'ok_start_nearby' if math.isfinite(start_snap_distance) else 'ok',
            (
                f'机器人位于小范围地形未知孔，已使用'
                f'{start_snap_distance:.2f} m外的可通行连通域验证目标'
                if math.isfinite(start_snap_distance)
                else '目标点可达'
            ),
            nearest_start_traversable_distance_m=start_snap_distance,
        )

    component_rows, component_columns = np.nonzero(
        grid.labels == start_label)
    if component_rows.size == 0:
        return GoalValidationResult(
            False,
            'robot_not_traversable',
            '机器人可通行连通域为空',
            nearest_start_traversable_distance_m=start_snap_distance,
        )
    centres = np.asarray([
        grid.cell_centre(row, column)
        for row, column in zip(component_rows, component_columns)
    ], dtype=np.float64)
    distances = np.hypot(
        centres[:, 0] - float(goal_x),
        centres[:, 1] - float(goal_y),
    )
    nearest = float(np.min(distances))
    if goal_label <= 0 and nearest <= float(goal_tolerance_m):
        return GoalValidationResult(
            True,
            'within_tolerance',
            '目标格不可通行，但同一连通域内存在容差范围内的替代格',
            nearest,
            start_snap_distance,
        )
    if goal_label > 0:
        return GoalValidationResult(
            False,
            'goal_disconnected',
            '目标点位于与机器人不连通的可通行区域',
            nearest,
            start_snap_distance,
        )
    return GoalValidationResult(
        False,
        'goal_blocked',
        '目标点为未知或禁止通行，且容差范围内没有同连通域替代格',
        nearest,
        start_snap_distance,
    )
