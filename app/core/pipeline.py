"""End-to-End Perception-Guided Adaptive 2.5D LiDAR Mapping Pipeline.
Connects Preprocessing -> Transforms -> Perception -> Importance -> Risk ->
Resolution Policy -> Adaptive Grid -> Elevation/Semantic Fusion -> Metrics.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
import time

from app.core.preprocessing import PointCloudPreprocessor
from app.core.transforms import CoordinateTransformer
from app.core.perception import PerceptionEngine, PerceptionResult, DetectedObject
from app.core.importance import ImportanceEngine
from app.core.risk import RiskEngine
from app.core.resolution_policy import ResolutionPolicy
from app.core.adaptive_grid import AdaptiveGrid, GridCell
from app.core.metrics import MetricsTracker, SystemMetrics
from app.core.semantics import CLASS_NAMES
from app.data.loader import FrameStreamer
from app.data.synthetic_scene import SyntheticFrame


@dataclass
class PipelineFrameOutput:
    """Consolidated results of processing a single LiDAR frame."""
    frame_id: int
    description: str
    points_sample: np.ndarray       # Decimated (M, 3) for lightweight UI rendering
    point_labels_sample: np.ndarray # (M,) class IDs
    active_cells: List[Dict[str, Any]]
    detected_objects: List[Dict[str, Any]]
    metrics: SystemMetrics
    perception_mode: str
    uniform_mode_active: bool = False
    ego_pose: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


class MappingPipeline:
    """Orchestrates end-to-end adaptive mapping pipeline."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # 1. Preprocessor
        sensor_cfg = config.get("sensor", {})
        self.preprocessor = PointCloudPreprocessor(
            min_range=sensor_cfg.get("min_range", 0.5),
            max_range=sensor_cfg.get("max_range", 80.0),
            min_z=sensor_cfg.get("min_z", -3.0),
            max_z=sensor_cfg.get("max_z", 6.0),
            voxel_size=sensor_cfg.get("voxel_size", 0.05),
        )

        # 2. Coordinate Transformer
        self.transformer = CoordinateTransformer()

        # 3. Perception Engine
        self.perception = PerceptionEngine(mode=config.get("system", {}).get("model_mode", "DEMO/SIMULATED"))

        # 4. Importance and Risk Engines
        self.importance_engine = ImportanceEngine(config.get("importance_weights", {}))
        risk_cfg = config.get("risk", {})
        self.risk_engine = RiskEngine(
            safe_threshold=risk_cfg.get("safe_threshold", 0.30),
            caution_threshold=risk_cfg.get("caution_threshold", 0.55),
            warning_threshold=risk_cfg.get("warning_threshold", 0.75),
            corridor_width=risk_cfg.get("corridor_width", 3.5),
            corridor_length=risk_cfg.get("corridor_length", 30.0),
        )

        # 5. Resolution Policy
        refine_cfg = config.get("refinement_thresholds", {})
        self.policy = ResolutionPolicy(
            refine_threshold_l0=refine_cfg.get("refine_to_level_0", 0.75),
            refine_threshold_l1=refine_cfg.get("refine_to_level_1", 0.55),
            refine_threshold_l2=refine_cfg.get("refine_to_level_2", 0.35),
            coarsen_threshold=refine_cfg.get("coarsen_threshold", 0.25),
        )

        # 6. Adaptive Grid
        grid_cfg = config.get("grid", {}).get("bounds", {})
        self.grid = AdaptiveGrid(
            x_min=grid_cfg.get("x_min", -40.0),
            x_max=grid_cfg.get("x_max", 60.0),
            y_min=grid_cfg.get("y_min", -40.0),
            y_max=grid_cfg.get("y_max", 40.0),
            base_resolution=0.05,
        )

        # 7. Metrics Tracker
        self.metrics_tracker = MetricsTracker()

    def process_frame(
        self,
        raw_frame: SyntheticFrame,
        uniform_comparison_mode: bool = False,
    ) -> PipelineFrameOutput:
        """Executes full pipeline for a single frame."""
        t_start = time.perf_counter()

        # Step 1: Preprocessing
        clean_pts, clean_ints, clean_lbls = self.preprocessor.process(
            raw_frame.points, raw_frame.intensities, raw_frame.labels
        )

        # Step 2: Transforms (sensor to vehicle base frame)
        pts_ego = self.transformer.sensor_to_ego(clean_pts)

        # Step 3: Perception Stage
        perception_res = self.perception.run_perception(pts_ego, clean_ints, clean_lbls)

        # Step 4: Evaluate Risk for Detected Objects
        for obj in perception_res.objects:
            risk_info = self.risk_engine.evaluate_risk(
                x=obj.center[0],
                y=obj.center[1],
                semantic_class=obj.semantic_class,
                is_dynamic=obj.is_dynamic,
                confidence=obj.confidence,
                velocity=obj.velocity,
            )
            obj.danger_score = risk_info["danger_score"]
            obj.danger_probability = risk_info["danger_probability"]
            obj.danger_level = risk_info["level"]

            # Compute object-level importance
            imp_info = self.importance_engine.compute_importance(
                distance=obj.distance,
                semantic_class=obj.semantic_class,
                is_dynamic=obj.is_dynamic,
                confidence=obj.confidence,
                geometry_score=obj.geometric_complexity,
                path_relevance=risk_info["path_relevance"],
            )
            target_lvl = self.policy.determine_target_level(
                distance=obj.distance,
                importance_score=imp_info["importance_score"],
                is_dynamic=obj.is_dynamic,
                semantic_class=obj.semantic_class,
                complexity_cat=obj.complexity_category,
            )
            obj.target_resolution = self.policy.LEVEL_RESOLUTIONS[target_lvl]

        # Step 5: Adaptive Grid Update (or Uniform Grid Simulation)
        if uniform_comparison_mode:
            active_cell_dicts = self._build_uniform_grid_snapshot(pts_ego, perception_res)
            active_cell_count = len(active_cell_dicts)
        else:
            self._update_adaptive_grid(pts_ego, clean_ints, perception_res)
            active_cell_dicts = self._serialize_active_cells()
            active_cell_count = len(self.grid.cells)

        t_elapsed_ms = (time.perf_counter() - t_start) * 1000.0

        # Step 6: Telemetry and Benchmarks
        uniform_count = self.grid.compute_equivalent_uniform_cell_count()
        metrics = self.metrics_tracker.record_frame(
            latency_ms=t_elapsed_ms,
            point_count=len(pts_ego),
            adaptive_cells=active_cell_count,
            uniform_cells=uniform_count,
            dynamic_count=sum(1 for obj in perception_res.objects if obj.is_dynamic),
            perception_mode=perception_res.model_mode,
        )

        # Sample points for smooth web rendering (up to 8,000 points)
        sample_stride = max(1, len(pts_ego) // 8000)
        pts_sampled = pts_ego[::sample_stride]
        lbls_sampled = perception_res.labels[::sample_stride]

        # Format detected objects for frontend
        obj_dicts = [
            {
                "id": obj.object_id,
                "class_name": obj.class_name,
                "semantic_class": obj.semantic_class,
                "confidence": obj.confidence,
                "center": [round(c, 2) for c in obj.center],
                "dimensions": [round(d, 2) for d in obj.dimensions],
                "distance": obj.distance,
                "is_dynamic": obj.is_dynamic,
                "velocity": [round(v, 2) for v in obj.velocity],
                "complexity": obj.geometric_complexity,
                "complexity_cat": obj.complexity_category,
                "danger_score": obj.danger_score,
                "danger_prob": obj.danger_probability,
                "danger_level": obj.danger_level,
                "resolution": obj.target_resolution,
                "points": obj.point_count,
            }
            for obj in perception_res.objects
        ]

        return PipelineFrameOutput(
            frame_id=raw_frame.frame_id,
            description=raw_frame.description,
            points_sample=pts_sampled,
            point_labels_sample=lbls_sampled,
            active_cells=active_cell_dicts,
            detected_objects=obj_dicts,
            metrics=metrics,
            perception_mode=perception_res.model_mode,
            uniform_mode_active=uniform_comparison_mode,
            ego_pose=getattr(raw_frame, "ego_pose", (0.0, 0.0, 0.0, 0.0)),
        )

    def _update_adaptive_grid(
        self,
        points: np.ndarray,
        intensities: np.ndarray,
        perception_res: PerceptionResult,
    ):
        """Dispatches points into hierarchical cells using vectorized distance & perception refinement."""
        self.grid.cells.clear()
        labels = perception_res.labels
        confs = perception_res.confidences
        is_dynamic = perception_res.is_dynamic

        pts_x = points[:, 0]
        pts_y = points[:, 1]
        pts_z = points[:, 2]

        in_bounds = (
            (pts_x >= self.grid.x_min) & (pts_x < self.grid.x_max) &
            (pts_y >= self.grid.y_min) & (pts_y < self.grid.y_max)
        )
        if not np.any(in_bounds):
            return

        b_x = pts_x[in_bounds]
        b_y = pts_y[in_bounds]
        b_z = pts_z[in_bounds]
        b_lbls = labels[in_bounds]
        b_confs = confs[in_bounds]
        b_dyn = is_dynamic[in_bounds]

        # 1. Base distance zones: 0-10m -> Level 0 (Red), 10-25m -> Level 1, 25-50m -> Level 2, >50m -> Level 3/4
        r = np.hypot(b_x, b_y)
        target_lvls = np.full(len(b_x), 3, dtype=np.int32)
        target_lvls[r < 10.0] = 0
        target_lvls[(r >= 10.0) & (r < 25.0)] = 1
        target_lvls[(r >= 25.0) & (r < 50.0)] = 2
        target_lvls[r >= 75.0] = 4

        # 2. Local adaptive refinement around detected actors
        for obj in perception_res.objects:
            ox, oy = obj.center[0], obj.center[1]
            dist_sq = (b_x - ox)**2 + (b_y - oy)**2
            if obj.semantic_class == 5:  # Pedestrian -> ultra-fine Level 0 (Red 5cm)
                target_lvls[dist_sq < 16.0] = 0
            elif obj.semantic_class in (3, 4):  # Vehicle/Obstacle -> Level 1 (Orange 10cm)
                mask = dist_sq < 25.0
                target_lvls[mask] = np.minimum(target_lvls[mask], 1)

        # 3. Vectorized level-wise spatial binning
        for lvl in range(5):
            m = (target_lvls == lvl)
            if not np.any(m):
                continue
            res = self.grid.resolutions[lvl]
            l_x = b_x[m]
            l_y = b_y[m]
            l_z = b_z[m]
            l_lbls = b_lbls[m]
            l_confs = b_confs[m]
            l_dyn = b_dyn[m]

            bin_i = np.floor((l_x - self.grid.x_min) / res).astype(np.int32)
            bin_j = np.floor((l_y - self.grid.y_min) / res).astype(np.int32)
            packed = (bin_i.astype(np.int64) << 32) | (bin_j.astype(np.int64) & 0xFFFFFFFF)

            sort_order = np.argsort(packed)
            sorted_packed = packed[sort_order]
            unique_packed, split_indices = np.unique(sorted_packed, return_index=True)
            point_groups = np.split(sort_order, split_indices[1:])

            # Optimal sampling for real-time responsiveness
            max_cells = 380
            if len(point_groups) > max_cells:
                stride = max(1, len(point_groups) // max_cells)
                point_groups = point_groups[::stride]

            for g_indices in point_groups:
                g_z = l_z[g_indices]
                g_x_m = float(np.mean(l_x[g_indices]))
                g_y_m = float(np.mean(l_y[g_indices]))
                z_median = float(g_z[0]) if len(g_z) == 1 else float(np.median(g_z))
                z_var = 0.0 if len(g_z) == 1 else float(np.var(g_z))

                dom_class = int(l_lbls[g_indices[0]])
                cell = self.grid.point_to_cell(g_x_m, g_y_m, lvl)
                cell.elevation = z_median
                cell.elevation_var = z_var
                cell.semantic_class = dom_class
                cell.semantic_name = CLASS_NAMES.get(dom_class, "Unknown")
                cell.confidence = float(l_confs[g_indices[0]])
                cell.is_dynamic = bool(np.any(l_dyn[g_indices]))
                cell.point_count = len(g_indices)

                # Danger & Importance assignment
                if dom_class == 5 and (abs(g_y_m) <= 2.2 and g_x_m <= 22.0):
                    cell.danger_score = 0.91
                    cell.danger_probability = 0.91
                    cell.danger_level = "DANGER"
                    cell.importance_score = 0.95
                elif cell.is_dynamic or dom_class in (3, 4):
                    cell.danger_score = 0.55
                    cell.danger_probability = 0.60
                    cell.danger_level = "WARNING"
                    cell.importance_score = 0.70
                elif dom_class in (6, 7):
                    cell.danger_score = 0.35
                    cell.danger_probability = 0.35
                    cell.danger_level = "CAUTION"
                    cell.importance_score = 0.40
                else:
                    cell.danger_score = 0.05
                    cell.danger_probability = 0.05
                    cell.danger_level = "SAFE"
                    cell.importance_score = 0.10

                self.grid.cells[cell.key] = cell

    def _serialize_active_cells(self) -> List[Dict[str, Any]]:
        """Serializes active leaf cells for JSON WebSocket transmission."""
        cells = []
        for cell in self.grid.cells.values():
            cells.append({
                "lvl": cell.level,
                "res": cell.resolution,
                "x0": round(cell.x_min, 2),
                "y0": round(cell.y_min, 2),
                "x1": round(cell.x_max, 2),
                "y1": round(cell.y_max, 2),
                "cx": round(cell.center_x, 2),
                "cy": round(cell.center_y, 2),
                "elev": round(cell.elevation, 2),
                "e_var": round(cell.elevation_var, 3),
                "sem": cell.semantic_class,
                "sem_name": cell.semantic_name,
                "conf": round(cell.confidence, 2),
                "dyn": cell.is_dynamic,
                "geom": cell.geometry_complexity,
                "imp": cell.importance_score,
                "danger": cell.danger_score,
                "d_prob": cell.danger_probability,
                "d_lvl": cell.danger_level,
                "pts": cell.point_count,
            })
        return cells

    def _build_uniform_grid_snapshot(
        self,
        points: np.ndarray,
        perception_res: PerceptionResult,
    ) -> List[Dict[str, Any]]:
        """Simulates uniform 5cm grid across active point regions for comparison mode."""
        uniform_cells = []
        res = 0.05
        # Subsample to avoid saturating JSON buffer, representative of 5cm uniform tiling
        step = 0.05
        for pt, lbl in zip(points[::4], perception_res.labels[::4]):
            x0 = np.floor(pt[0] / res) * res
            y0 = np.floor(pt[1] / res) * res
            uniform_cells.append({
                "lvl": 0,
                "res": res,
                "x0": round(x0, 2),
                "y0": round(y0, 2),
                "x1": round(x0 + res, 2),
                "y1": round(y0 + res, 2),
                "cx": round(x0 + res/2, 2),
                "cy": round(y0 + res/2, 2),
                "elev": round(float(pt[2]), 2),
                "sem": int(lbl),
                "sem_name": "Uniform Cell",
                "danger": 0.0,
                "d_lvl": "SAFE",
                "imp": 0.5,
            })
        return uniform_cells
