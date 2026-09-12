"""Script to pre-render OCULUS sequential telemetry frames for static deployment."""
import os
import json
import time
import yaml
from pathlib import Path

from app.core.pipeline import MappingPipeline
from app.data.loader import FrameStreamer

def export_all():
    cfg_path = Path("config/config.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    pipeline = MappingPipeline(cfg)
    streamer = FrameStreamer()

    static_api = Path("app/visualization/static/api")
    frames_dir = static_api / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # 1. Status and World Map
    status_data = {
        "system": "OCULUS",
        "model_mode": "PRECOMPUTED (MODULAR ARCHITECTURE)",
        "total_frames": streamer.total_frames,
        "is_synthetic": streamer.is_synthetic,
        "world_map": streamer.get_world_map()
    }
    with open(static_api / "status.json", "w", encoding="utf-8") as f:
        json.dump(status_data, f, indent=2)
    with open(static_api / "world_map.json", "w", encoding="utf-8") as f:
        json.dump(streamer.get_world_map(), f, indent=2)

    total = min(streamer.total_frames, 180)
    print(f"Exporting {total} adaptive telemetry frames...")
    t0 = time.time()

    for fid in range(1, total + 1):
        raw = streamer.get_frame(fid - 1)
        out = pipeline.process_frame(raw, uniform_comparison_mode=False)

        pts = [[round(float(c), 2) for c in pt] for pt in out.points_sample[::2]]
        lbls = [int(l) for l in out.point_labels_sample[::2]]

        cells = []
        for c in out.active_cells:
            cells.append({
                "lvl": int(c["lvl"]),
                "res": float(c["res"]),
                "cx": round(float(c["cx"]), 2),
                "cy": round(float(c["cy"]), 2),
                "x0": round(float(c.get("x0", c["cx"] - c["res"]/2)), 2),
                "y0": round(float(c.get("y0", c["cy"] - c["res"]/2)), 2),
                "x1": round(float(c.get("x1", c["cx"] + c["res"]/2)), 2),
                "y1": round(float(c.get("y1", c["cy"] + c["res"]/2)), 2),
                "elev": round(float(c["elev"]), 2),
                "e_var": round(float(c.get("e_var", 0.035)), 3),
                "sem": int(c["sem"]),
                "sem_name": str(c["sem_name"]),
                "conf": round(float(c.get("conf", 0.92)), 2),
                "danger": round(float(c["danger"]), 2),
                "d_lvl": str(c["d_lvl"]),
                "d_prob": round(float(c.get("d_prob", c["danger"])), 2),
                "imp": round(float(c["imp"]), 2),
                "pts": int(c.get("pts", 14)),
            })

        frame_dict = {
            "frame_id": out.frame_id,
            "phase_name": getattr(raw, "phase_name", "Urban Cruising"),
            "description": out.description,
            "ego_pose": [round(float(x), 3) for x in getattr(out, "ego_pose", (0.0, 0.0, 0.0, 0.0))],
            "speed_mps": round(float(getattr(raw, "speed_mps", 4.0)), 2),
            "timestamp_s": round(float(getattr(raw, "timestamp_s", float(fid))), 2),
            "perception_mode": "PRECOMPUTED (MODULAR ARCHITECTURE)",
            "uniform_mode_active": False,
            "points_sample": pts,
            "point_labels_sample": lbls,
            "active_cells": cells,
            "detected_objects": out.detected_objects,
            "metrics": {
                "fps": out.metrics.fps,
                "latency_ms": out.metrics.latency_ms,
                "point_count": out.metrics.point_count,
                "active_adaptive_cells": out.metrics.active_adaptive_cells,
                "theoretical_uniform_cells": out.metrics.theoretical_uniform_cells,
                "cell_reduction_percent": out.metrics.cell_reduction_percent,
                "estimated_adaptive_storage_kb": out.metrics.estimated_adaptive_storage_kb,
                "estimated_uniform_storage_kb": out.metrics.estimated_uniform_storage_kb,
                "dynamic_object_count": out.metrics.dynamic_object_count,
                "current_frame": out.metrics.current_frame,
            }
        }

        with open(frames_dir / f"frame_{fid}.json", "w", encoding="utf-8") as f:
            json.dump(frame_dict, f, separators=(",", ":"))

    print(f"Export completed in {time.time()-t0:.1f}s")

if __name__ == "__main__":
    export_all()
