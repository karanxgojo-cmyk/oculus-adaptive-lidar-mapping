"""Dataset Loader & Point Cloud Streaming Interface.
Loads SemanticKITTI .bin LiDAR scans and .label files, or streams synthetic frames.
"""

from pathlib import Path
from typing import Any, Dict, Generator, List, Optional, Tuple
import numpy as np

from app.data.synthetic_scene import SyntheticLiDARGenerator, SyntheticFrame


class PointCloudLoader:
    """Reads binary point clouds and streams frames."""

    @staticmethod
    def load_kitti_bin(bin_path: str) -> Tuple[np.ndarray, np.ndarray]:
        """Loads SemanticKITTI .bin point cloud (float32 x, y, z, intensity).
        
        Returns:
            points: (N, 3) float32
            intensities: (N,) float32
        """
        scan = np.fromfile(bin_path, dtype=np.float32)
        if len(scan) % 4 != 0:
            raise ValueError(f"File {bin_path} length {len(scan)} is not divisible by 4.")
        scan = scan.reshape((-1, 4))
        points = scan[:, :3].astype(np.float32)
        intensities = scan[:, 3].astype(np.float32)
        return points, intensities

    @staticmethod
    def load_kitti_label(label_path: str) -> Tuple[np.ndarray, np.ndarray]:
        """Loads SemanticKITTI .label file (uint32).
        Lower 16 bits = semantic label. Upper 16 bits = instance ID.
        
        Returns:
            semantic_labels: (N,) uint32 mapped to our 8-class system
            instance_ids: (N,) uint32
        """
        raw_labels = np.fromfile(label_path, dtype=np.uint32)
        sem_labels = raw_labels & 0xFFFF
        inst_ids = raw_labels >> 16

        # Map SemanticKITTI classes to our unified 8 classes:
        # 0: Unknown, 1: Drivable, 2: Non-Drivable, 3: Obstacle, 4: Vehicle, 5: Pedestrian, 6: Pole, 7: Wall
        kitti_map = {
            # Road / lane-marking / parking
            40: 1, 44: 1, 48: 1, 60: 1,
            # Sidewalk / other-ground / terrain
            49: 2, 72: 2, 70: 2, 71: 2,
            # Building / fence / trunk / vegetation
            50: 7, 51: 7, 70: 3, 80: 6, 81: 3,
            # Car / truck / bicycle / motorcycle
            10: 4, 11: 4, 13: 4, 15: 4, 16: 4, 18: 4, 20: 4,
            # Person / bicyclist / motorcyclist
            30: 5, 31: 5, 32: 5,
            # Pole / traffic-sign
            80: 6, 99: 6,
        }

        unified_labels = np.zeros(len(sem_labels), dtype=np.uint32)
        for k, v in kitti_map.items():
            unified_labels[sem_labels == k] = v

        return unified_labels, inst_ids


class FrameStreamer:
    """Streams LiDAR frames from a directory or synthetic generator."""

    def __init__(self, data_dir: Optional[str] = None):
        self.data_dir = Path(data_dir) if data_dir else None
        self.bin_files: List[Path] = []
        self.synthetic_gen = SyntheticLiDARGenerator()
        self.synthetic_frames = self.synthetic_gen.generate_3min_sequence(num_frames=180)

        if self.data_dir and self.data_dir.exists():
            self.bin_files = sorted(list(self.data_dir.glob("*.bin")))

    @property
    def is_synthetic(self) -> bool:
        return len(self.bin_files) == 0

    @property
    def total_frames(self) -> int:
        return len(self.bin_files) if self.bin_files else len(self.synthetic_frames)

    def get_world_map(self) -> Dict[str, Any]:
        """Returns structural world map specifications (roads, intersections, crosswalks, buildings)."""
        return self.synthetic_gen.get_world_map_spec()

    def get_frame(self, index: int) -> SyntheticFrame:
        """Retrieves a specific frame by index (0-indexed)."""
        if self.is_synthetic:
            idx = index % len(self.synthetic_frames)
            return self.synthetic_frames[idx]

        # Load real KITTI frame
        idx = index % len(self.bin_files)
        bin_p = self.bin_files[idx]
        pts, ints = PointCloudLoader.load_kitti_bin(str(bin_p))
        
        lbl_p = bin_p.with_suffix(".label")
        if lbl_p.exists():
            lbls, _ = PointCloudLoader.load_kitti_label(str(lbl_p))
        else:
            lbls = np.zeros(len(pts), dtype=np.uint32)

        return SyntheticFrame(
            frame_id=index + 1,
            description=f"SemanticKITTI Bin Frame: {bin_p.name}",
            points=pts,
            intensities=ints,
            labels=lbls,
            ego_pose=(0.0, 0.0, 0.0, 0.0),
        )
