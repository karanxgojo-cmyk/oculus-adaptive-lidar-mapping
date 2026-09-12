"""LiDAR point cloud preprocessing module.
Handles invalid point removal, range and height gating, and voxel downsampling.
"""

from typing import Optional, Tuple
import numpy as np


class PointCloudPreprocessor:
    """Preprocesses raw LiDAR point clouds (x, y, z, intensity, optional features)."""

    def __init__(
        self,
        min_range: float = 0.5,
        max_range: float = 80.0,
        min_z: float = -3.0,
        max_z: float = 6.0,
        voxel_size: Optional[float] = None,
    ):
        self.min_range = min_range
        self.max_range = max_range
        self.min_z = min_z
        self.max_z = max_z
        self.voxel_size = voxel_size

    def process(
        self,
        points: np.ndarray,
        intensities: Optional[np.ndarray] = None,
        labels: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
        """Preprocesses point cloud array of shape (N, 3).
        
        Returns:
            clean_points: (M, 3) float32
            clean_intensities: (M,) float32
            clean_labels: (M,) uint32 or None
        """
        if len(points) == 0:
            return (
                np.zeros((0, 3), dtype=np.float32),
                np.zeros((0,), dtype=np.float32),
                None if labels is None else np.zeros((0,), dtype=np.uint32),
            )

        # 1. Check finite values
        valid_mask = np.isfinite(points).all(axis=1)
        if intensities is not None:
            valid_mask &= np.isfinite(intensities)

        # 2. Range filtering
        r = np.linalg.norm(points[:, :2], axis=1)  # 2D ground radial distance
        valid_mask &= (r >= self.min_range) & (r <= self.max_range)

        # 3. Z-height gating
        valid_mask &= (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z)

        clean_pts = points[valid_mask].astype(np.float32)
        clean_ints = (
            intensities[valid_mask].astype(np.float32)
            if intensities is not None
            else np.ones(len(clean_pts), dtype=np.float32)
        )
        clean_lbls = labels[valid_mask].astype(np.uint32) if labels is not None else None

        # 4. Optional Voxel Grid Downsampling
        if self.voxel_size and self.voxel_size > 0 and len(clean_pts) > 0:
            clean_pts, clean_ints, clean_lbls = self._voxel_downsample(
                clean_pts, clean_ints, clean_lbls, self.voxel_size
            )

        return clean_pts, clean_ints, clean_lbls

    @staticmethod
    def _voxel_downsample(
        points: np.ndarray,
        intensities: np.ndarray,
        labels: Optional[np.ndarray],
        voxel_size: float,
    ) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
        """Fast grid-voxel decimation using unique voxel coordinates."""
        coords = np.floor(points / voxel_size).astype(np.int64)
        # Construct unique 1D hash keys
        _, unique_indices = np.unique(coords, axis=0, return_index=True)
        return (
            points[unique_indices],
            intensities[unique_indices],
            labels[unique_indices] if labels is not None else None,
        )
