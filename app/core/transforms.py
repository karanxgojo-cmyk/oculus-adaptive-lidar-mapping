"""Coordinate frame transformation utilities for AV and LiDAR systems.
Handles rigid transforms (SE3), ego-vehicle frame to world frame, and forward projections.
"""

from typing import Tuple
import numpy as np


class CoordinateTransformer:
    """Handles transformations between Sensor, Vehicle (Ego), and Map/World frames."""

    def __init__(
        self,
        sensor_to_ego_translation: Tuple[float, float, float] = (0.0, 0.0, 1.73),
        sensor_to_ego_rotation_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        """Initializes sensor to ego transform.
        Default: Velodyne mounted at x=0.0m, y=0.0m, z=1.73m height with 0 roll/pitch/yaw.
        """
        self.T_sensor_to_ego = self._build_transform_matrix(
            sensor_to_ego_translation, sensor_to_ego_rotation_rpy
        )

    @staticmethod
    def _build_transform_matrix(
        translation: Tuple[float, float, float],
        rpy: Tuple[float, float, float],
    ) -> np.ndarray:
        """Constructs a 4x4 SE(3) homogeneous matrix from translation and Roll-Pitch-Yaw."""
        r, p, y = rpy
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)

        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]
        ], dtype=np.float32)

        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[:3, 3] = translation
        return T

    def transform_points(self, points: np.ndarray, transform_matrix: np.ndarray) -> np.ndarray:
        """Applies a 4x4 transform to (N, 3) points."""
        if len(points) == 0:
            return points
        homo = np.hstack([points, np.ones((len(points), 1), dtype=np.float32)])
        transformed = (transform_matrix @ homo.T).T
        return transformed[:, :3]

    def sensor_to_ego(self, points: np.ndarray) -> np.ndarray:
        """Transforms points from sensor frame to ego vehicle frame."""
        return self.transform_points(points, self.T_sensor_to_ego)

    def ego_to_world(
        self,
        points: np.ndarray,
        ego_pose: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
    ) -> np.ndarray:
        """Transforms points from ego vehicle frame to world frame given ego pose (x, y, z, yaw)."""
        x, y, z, yaw = ego_pose
        T_ego_to_world = self._build_transform_matrix((x, y, z), (0.0, 0.0, yaw))
        return self.transform_points(points, T_ego_to_world)
