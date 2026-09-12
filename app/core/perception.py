"""Unified Perception Engine & Object Detection Coordinator.
Interfaces with model adapters (PointNet++, PolarNet, Cylinder3D, CenterPoint, Patchwork++)
and provides high-fidelity deterministic geometric clustering when running in DEMO mode.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import numpy as np
import time

from app.core.semantics import CLASS_NAMES, CLASS_COLORS


@dataclass
class DetectedObject:
    """Represents a 3D detected obstacle/actor in the scene."""
    object_id: int
    semantic_class: int
    class_name: str
    confidence: float
    center: Tuple[float, float, float]  # (x, y, z)
    dimensions: Tuple[float, float, float]  # (dx, dy, dz)
    distance: float
    is_dynamic: bool
    velocity: Tuple[float, float]  # (vx, vy)
    geometric_complexity: float
    complexity_category: str
    danger_score: float = 0.0
    danger_probability: float = 0.0
    danger_level: str = "SAFE"
    target_resolution: float = 0.05
    point_count: int = 0


@dataclass
class PerceptionResult:
    """Aggregated perception output for a single LiDAR frame."""
    points: np.ndarray                 # (N, 3) XYZ coordinates
    labels: np.ndarray                 # (N,) semantic class IDs
    confidences: np.ndarray            # (N,) prediction confidence [0..1]
    is_dynamic: np.ndarray             # (N,) bool dynamic mask
    objects: List[DetectedObject]      # Clustered actors/obstacles
    model_name: str
    model_mode: str                    # "REAL" or "DEMO/SIMULATED"
    latency_ms: float


class PerceptionEngine:
    """Coordinates perception models and clustering algorithms."""

    def __init__(self, primary_adapter=None, mode: str = "DEMO/SIMULATED"):
        self.adapter = primary_adapter
        self.mode = mode
        self.next_object_id = 1

    def run_perception(
        self,
        points: np.ndarray,
        intensities: Optional[np.ndarray] = None,
        ground_truth_labels: Optional[np.ndarray] = None,
    ) -> PerceptionResult:
        """Executes perception inference on points.
        Uses attached adapter if available; otherwise falls back to deterministic clustering.
        """
        t0 = time.perf_counter()

        if self.adapter is not None:
            res = self.adapter.predict(points, intensities, ground_truth_labels)
            res.latency_ms = (time.perf_counter() - t0) * 1000.0
            return res

        # Fallback: High-fidelity geometric/label perception pipeline
        if ground_truth_labels is not None and len(ground_truth_labels) == len(points):
            labels = ground_truth_labels.copy()
            confidences = np.random.uniform(0.85, 0.99, size=len(points)).astype(np.float32)
        else:
            labels, confidences = self._heuristic_segmentation(points)

        # Dynamic mask determination (Pedestrians and moving vehicles)
        is_dynamic = (labels == 5) | ((labels == 4) & (points[:, 0] > 5.0) & (points[:, 0] < 45.0))

        # Cluster non-ground objects into discrete 3D instances
        objects = self._cluster_objects(points, labels, confidences, is_dynamic)

        latency_ms = (time.perf_counter() - t0) * 1000.0

        return PerceptionResult(
            points=points,
            labels=labels,
            confidences=confidences,
            is_dynamic=is_dynamic,
            objects=objects,
            model_name="Deterministic Perception Pipeline (SIH Modular)",
            model_mode="DEMO/SIMULATED",
            latency_ms=round(latency_ms, 2),
        )

    def _heuristic_segmentation(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Simple geometric rule-based segmenter for raw unannotated points."""
        labels = np.zeros(len(points), dtype=np.uint32)
        confidences = np.full(len(points), 0.90, dtype=np.float32)

        # Road / ground: points with z < -1.4m
        ground_mask = points[:, 2] < -1.4
        labels[ground_mask] = 1  # Drivable road

        # Curb / sidewalk
        curb_mask = (points[:, 2] >= -1.4) & (points[:, 2] < -1.2)
        labels[curb_mask] = 2  # Non-drivable

        # Objects above ground
        elevated_mask = points[:, 2] >= -1.2
        elev_pts = points[elevated_mask]
        
        # Distinguish poles, walls, obstacles
        for idx in np.where(elevated_mask)[0]:
            p = points[idx]
            dist_lat = abs(p[1])
            if dist_lat > 12.0:
                labels[idx] = 7  # Wall
            elif p[2] > 1.5 and dist_lat > 3.0:
                labels[idx] = 6  # Pole
            elif abs(p[1]) < 3.5 and p[0] > 0:
                labels[idx] = 4  # Vehicle
            else:
                labels[idx] = 3  # Static Obstacle

        return labels, confidences

    def _cluster_objects(
        self,
        points: np.ndarray,
        labels: np.ndarray,
        confidences: np.ndarray,
        is_dynamic: np.ndarray,
    ) -> List[DetectedObject]:
        """Spatial clustering to identify 3D bounding boxes and actor states."""
        objects = []
        # Target classes to cluster: Obstacle (3), Vehicle (4), Pedestrian (5), Pole (6), Wall (7)
        target_classes = [3, 4, 5, 6, 7]

        for cls_id in target_classes:
            mask = (labels == cls_id)
            if not np.any(mask):
                continue

            cls_points = points[mask]
            cls_confs = confidences[mask]
            cls_dyn = is_dynamic[mask]

            if len(cls_points) < 5:
                continue

            # Grid-based fast spatial clustering
            cluster_voxel = 1.0 if cls_id == 5 else 2.5
            coarse_coords = np.floor(cls_points[:, :2] / cluster_voxel).astype(np.int32)
            unique_bins, inv_indices = np.unique(coarse_coords, axis=0, return_inverse=True)

            for b_idx in range(len(unique_bins)):
                bin_mask = (inv_indices == b_idx)
                pts_in_cluster = cls_points[bin_mask]
                if len(pts_in_cluster) < 8:
                    continue

                min_xyz = np.min(pts_in_cluster, axis=0)
                max_xyz = np.max(pts_in_cluster, axis=0)
                center_xyz = (min_xyz + max_xyz) / 2.0
                dims = max_xyz - min_xyz

                dist = float(np.linalg.norm(center_xyz[:2]))
                dyn_flag = bool(np.mean(cls_dyn[bin_mask]) > 0.4)
                avg_conf = float(np.mean(cls_confs[bin_mask]))

                # Velocity vector simulation
                if cls_id == 5 and dyn_flag:
                    # Pedestrian walking across
                    vel = (-0.8, -1.2)
                elif cls_id == 4 and dyn_flag:
                    # Vehicle moving along lane
                    vel = (-8.5, 0.0)
                else:
                    vel = (0.0, 0.0)

                # Geometric complexity score
                density = min(len(pts_in_cluster) / 100.0, 1.0)
                height_spread = min((max_xyz[2] - min_xyz[2]) / 2.0, 1.0)
                raw_geom = 0.5 * density + 0.5 * height_spread
                geom_score = round(float(raw_geom), 3)
                geom_cat = "HIGH" if geom_score > 0.65 else ("MEDIUM" if geom_score > 0.35 else "LOW")

                obj = DetectedObject(
                    object_id=self.next_object_id,
                    semantic_class=cls_id,
                    class_name=CLASS_NAMES.get(cls_id, "Unknown"),
                    confidence=round(avg_conf, 3),
                    center=(round(float(center_xyz[0]), 2), round(float(center_xyz[1]), 2), round(float(center_xyz[2]), 2)),
                    dimensions=(round(float(dims[0]), 2), round(float(dims[1]), 2), round(float(dims[2]), 2)),
                    distance=round(dist, 2),
                    is_dynamic=dyn_flag,
                    velocity=vel,
                    geometric_complexity=geom_score,
                    complexity_category=geom_cat,
                    point_count=len(pts_in_cluster),
                )
                self.next_object_id += 1
                objects.append(obj)

        return objects
