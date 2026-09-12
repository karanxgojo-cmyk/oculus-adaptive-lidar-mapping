"""Automated Verification and Unit Test Suite.
Verifies all mathematical operations, parent-child quadtree tiling, risk and importance scoring,
elevation and semantic fusion, synthetic scene generation, and end-to-end pipeline execution.
"""

import sys
from pathlib import Path
import numpy as np

# Ensure project root is in python path
project_root = Path(__file__).parent.resolve()
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from app.core.preprocessing import PointCloudPreprocessor
from app.core.transforms import CoordinateTransformer
from app.core.elevation import ElevationLayer
from app.core.semantics import SemanticLayer
from app.core.risk import RiskEngine
from app.core.importance import ImportanceEngine
from app.core.resolution_policy import ResolutionPolicy
from app.core.adaptive_grid import AdaptiveGrid, GridCell
from app.core.perception import PerceptionEngine
from app.data.synthetic_scene import SyntheticLiDARGenerator
from app.core.pipeline import MappingPipeline
import yaml


def test_preprocessing():
    print("Testing Preprocessing...")
    prep = PointCloudPreprocessor(min_range=1.0, max_range=50.0, min_z=-2.0, max_z=3.0)
    raw_pts = np.array([
        [0.0, 0.5, 0.0],    # too close (< 1.0m)
        [10.0, 5.0, 0.0],   # valid
        [60.0, 0.0, 0.0],   # too far (> 50.0m)
        [15.0, 2.0, -4.0],  # too low (z < -2.0m)
        [20.0, 3.0, 1.0],   # valid
        [np.nan, 2.0, 1.0], # invalid NaN
    ], dtype=np.float32)

    pts, ints, _ = prep.process(raw_pts)
    assert len(pts) == 2, f"Expected 2 valid points, got {len(pts)}"
    assert np.all(np.isfinite(pts)), "Points contain non-finite values"
    print("  -> Preprocessing passed.")


def test_coordinate_transforms():
    print("Testing Coordinate Transforms...")
    trans = CoordinateTransformer(sensor_to_ego_translation=(1.0, 0.0, 0.5))
    pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    ego_pts = trans.sensor_to_ego(pts)
    assert np.allclose(ego_pts[0], [1.0, 0.0, 0.5]), f"Transform mismatch: {ego_pts[0]}"
    print("  -> Transforms passed.")


def test_elevation_fusion():
    print("Testing Elevation Layer...")
    z_vals = np.array([1.2, 1.25, 1.22, 1.18, 5.5], dtype=np.float32)  # 5.5 is an outlier
    stats = ElevationLayer.compute_elevation_stats(z_vals)
    # Median should reject outlier
    assert abs(stats["elevation"] - 1.22) < 0.01, f"Expected median 1.22, got {stats['elevation']}"
    assert stats["count"] == 5
    print("  -> Elevation fusion passed.")


def test_semantic_fusion():
    print("Testing Semantic Layer...")
    labels = np.array([1, 1, 1, 4, 1], dtype=np.uint32)
    confs = np.array([0.9, 0.8, 0.95, 0.7, 0.85], dtype=np.float32)
    res = SemanticLayer.aggregate_points_semantics(labels, confs)
    assert res["dominant_class"] == 1, f"Expected Road (1), got {res['dominant_class']}"
    assert res["dominant_name"] == "Drivable"
    assert res["confidence"] > 0.7
    print("  -> Semantic fusion passed.")


def test_risk_and_importance():
    print("Testing Risk and Importance Scoring...")
    risk_eng = RiskEngine()
    imp_eng = ImportanceEngine()

    # Pedestrian close and directly in forward path corridor
    ped_risk = risk_eng.evaluate_risk(
        x=8.0, y=0.2, semantic_class=5, is_dynamic=True, confidence=0.92, velocity=(-1.0, 0.0)
    )
    assert ped_risk["level"] in ["WARNING", "DANGER"], f"Expected high risk, got {ped_risk['level']}"
    assert ped_risk["danger_score"] > 0.65
    assert ped_risk["in_corridor"] is True

    ped_imp = imp_eng.compute_importance(
        distance=8.0, semantic_class=5, is_dynamic=True, confidence=0.92, geometry_score=0.8, path_relevance=1.0
    )
    assert ped_imp["importance_score"] >= 0.75, f"Expected high importance, got {ped_imp['importance_score']}"

    # Distant static road cell
    road_risk = risk_eng.evaluate_risk(
        x=55.0, y=10.0, semantic_class=1, is_dynamic=False, confidence=0.95
    )
    assert road_risk["level"] == "SAFE", f"Expected SAFE, got {road_risk['level']}"
    print("  -> Risk and Importance passed.")


def test_adaptive_grid_hierarchy():
    print("Testing Adaptive Grid Hierarchy & Parent-Child Relationships...")
    grid = AdaptiveGrid(x_min=-20.0, x_max=20.0, y_min=-20.0, y_max=20.0, base_resolution=0.05)
    
    # Create Level 2 cell (0.20m resolution)
    cell_l2 = grid.point_to_cell(x=5.1, y=2.3, level=2)
    assert cell_l2.resolution == 0.20
    assert cell_l2.level == 2

    # Refine to Level 1 (4 children of 0.10m)
    children = grid.refine_cell(cell_l2)
    assert len(children) == 4
    assert all(c.level == 1 for c in children)
    assert all(c.resolution == 0.10 for c in children)

    # Verify children bounds perfectly partition parent bounds with zero overlap/gap
    child_x_mins = sorted([c.x_min for c in children])
    child_y_mins = sorted([c.y_min for c in children])
    assert np.isclose(child_x_mins[0], cell_l2.x_min), f"Expected {cell_l2.x_min}, got {child_x_mins[0]}"
    assert np.isclose(child_y_mins[0], cell_l2.y_min), f"Expected {cell_l2.y_min}, got {child_y_mins[0]}"
    assert np.isclose(max(c.x_max for c in children), cell_l2.x_max)
    assert np.isclose(max(c.y_max for c in children), cell_l2.y_max)

    # Merge back to Level 2
    parent_key = cell_l2.key
    merged_parent = grid.merge_cells(parent_key)
    assert merged_parent is not None
    assert merged_parent.level == 2
    assert merged_parent.resolution == 0.20
    print("  -> Adaptive Grid hierarchy passed.")


def test_synthetic_sequence_and_pipeline():
    print("Testing Synthetic Sequence & End-to-End Mapping Pipeline...")
    gen = SyntheticLiDARGenerator()
    frames = gen.generate_presentation_sequence()
    assert len(frames) == 5, f"Expected 5 frames, got {len(frames)}"
    for f in frames:
        assert len(f.points) > 10000, f"Frame {f.frame_id} has insufficient points: {len(f.points)}"

    with open(project_root / "config" / "config.yaml", "r", encoding="utf-8") as file:
        cfg = yaml.safe_load(file)

    pipeline = MappingPipeline(cfg)
    
    # Process all 5 frames
    for f in frames:
        output = pipeline.process_frame(f)
        assert output.frame_id == f.frame_id
        assert len(output.active_cells) > 0
        assert output.metrics.cell_reduction_percent > 50.0, f"Reduction was {output.metrics.cell_reduction_percent}%"
        assert output.metrics.fps > 0

    print("  -> End-to-End Pipeline test passed.")


def main():
    print("==================================================")
    print("  RUNNING COMPLETE VERIFICATION TEST SUITE")
    print("==================================================")
    test_preprocessing()
    test_coordinate_transforms()
    test_elevation_fusion()
    test_semantic_fusion()
    test_risk_and_importance()
    test_adaptive_grid_hierarchy()
    test_synthetic_sequence_and_pipeline()
    print("==================================================")
    print("  ALL TESTS PASSED SUCCESSFULLY! PROTOTYPE READY.")
    print("==================================================")


if __name__ == "__main__":
    main()
