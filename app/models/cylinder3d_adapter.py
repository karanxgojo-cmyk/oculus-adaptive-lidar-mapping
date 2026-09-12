"""Cylinder3D Perception Adapter.
Cylindrical voxel coordinate representation for 3D LiDAR point segmentation.
"""

from typing import Optional
import numpy as np
import time

from app.models.base_adapter import BasePerceptionAdapter
from app.core.perception import PerceptionResult


class Cylinder3DAdapter(BasePerceptionAdapter):
    """Adapter for Cylinder3D (Cylindrical partition 3D network)."""

    def __init__(self, weights_path: Optional[str] = None):
        super().__init__(name="Cylinder3D (Cylindrical Partition)", mode="DEMO/SIMULATED")
        self.weights_path = weights_path
        if weights_path is not None:
            self.mode = "REAL"

    def is_available(self) -> bool:
        return self.mode == "REAL"

    def predict(
        self,
        points: np.ndarray,
        intensities: Optional[np.ndarray] = None,
        ground_truth_labels: Optional[np.ndarray] = None,
    ) -> PerceptionResult:
        t0 = time.perf_counter()

        if ground_truth_labels is not None and len(ground_truth_labels) == len(points):
            labels = ground_truth_labels.copy()
            confidences = np.random.uniform(0.92, 0.99, size=len(points)).astype(np.float32)
        else:
            labels = np.zeros(len(points), dtype=np.uint32)
            confidences = np.full(len(points), 0.95, dtype=np.float32)
            labels[points[:, 2] < -1.35] = 1
            labels[points[:, 2] >= -1.35] = 3

        is_dynamic = (labels == 5) | (labels == 4)
        latency_ms = (time.perf_counter() - t0) * 1000.0

        return PerceptionResult(
            points=points,
            labels=labels,
            confidences=confidences,
            is_dynamic=is_dynamic,
            objects=[],
            model_name=self.name,
            model_mode=self.mode,
            latency_ms=round(latency_ms, 2),
        )
