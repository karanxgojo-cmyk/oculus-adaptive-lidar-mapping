"""FastAPI Web Dashboard Server for Perception-Guided Adaptive 2.5D Mapping.
Hosts cyber-minimal AV visualizer, delivers real-time frame telemetry, and supports 1-click presentation demo.
"""

import asyncio
import json
from pathlib import Path
import threading
from typing import Any, Dict, Optional, Tuple
import webbrowser

from fastapi import FastAPI, Query, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from app.core.pipeline import MappingPipeline, PipelineFrameOutput
from app.data.loader import FrameStreamer


class DashboardServer:
    """FastAPI-based dashboard server."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.pipeline = MappingPipeline(config)
        self.streamer = FrameStreamer()
        self.host = config.get("dashboard", {}).get("host", "127.0.0.1")
        self.port = config.get("dashboard", {}).get("port", 8050)
        self.auto_open = config.get("dashboard", {}).get("auto_open_browser", True)

        self.json_cache: Dict[Tuple[int, bool], str] = {}
        self.app = FastAPI(title="OCULUS - Perception-Guided Adaptive 2.5D LiDAR Mapping")
        self._setup_routes()

        # Synchronously precompute Frame 1 so initial load has 0.0ms delay
        self._compute_and_cache_frame(1, False)

        # Pre-warm remaining frames asynchronously in background
        threading.Thread(target=self._prewarm_cache, daemon=True).start()

    def _compute_and_cache_frame(self, frame_id: int, uniform: bool) -> str:
        """Computes, serializes, and caches frame JSON for instant delivery."""
        cache_key = (frame_id, uniform)
        if cache_key in self.json_cache:
            return self.json_cache[cache_key]

        raw_frame = self.streamer.get_frame(frame_id - 1)
        output = self.pipeline.process_frame(raw_frame, uniform_comparison_mode=uniform)

        pts_list = output.points_sample.tolist() if hasattr(output.points_sample, "tolist") else []
        lbls_list = output.point_labels_sample.tolist() if hasattr(output.point_labels_sample, "tolist") else []

        res = {
            "frame_id": output.frame_id,
            "phase_name": getattr(raw_frame, "phase_name", "Urban Cruising"),
            "description": output.description,
            "ego_pose": list(getattr(output, "ego_pose", (0.0, 0.0, 0.0, 0.0))),
            "speed_mps": getattr(raw_frame, "speed_mps", 4.0),
            "timestamp_s": getattr(raw_frame, "timestamp_s", float(frame_id)),
            "perception_mode": "PRECOMPUTED (MODULAR ARCHITECTURE)",
            "uniform_mode_active": output.uniform_mode_active,
            "points_sample": pts_list,
            "point_labels_sample": lbls_list,
            "active_cells": output.active_cells,
            "detected_objects": output.detected_objects,
            "metrics": {
                "fps": output.metrics.fps,
                "latency_ms": output.metrics.latency_ms,
                "point_count": output.metrics.point_count,
                "active_adaptive_cells": output.metrics.active_adaptive_cells,
                "theoretical_uniform_cells": output.metrics.theoretical_uniform_cells,
                "cell_reduction_percent": output.metrics.cell_reduction_percent,
                "estimated_adaptive_storage_kb": output.metrics.estimated_adaptive_storage_kb,
                "estimated_uniform_storage_kb": output.metrics.estimated_uniform_storage_kb,
                "dynamic_object_count": output.metrics.dynamic_object_count,
                "current_frame": output.metrics.current_frame,
            },
        }
        json_str = json.dumps(res, default=lambda o: float(o) if hasattr(o, "item") else str(o))
        self.json_cache[cache_key] = json_str
        return json_str

    def _prewarm_cache(self):
        """Precomputes frames into memory so live playback has strictly <1ms server latency."""
        import time
        num_frames = min(self.streamer.total_frames, 180)
        for fid in range(2, num_frames + 1):
            try:
                self._compute_and_cache_frame(fid, False)
                time.sleep(0.005)
            except Exception as e:
                print(f"Pre-warm frame {fid} error: {e}")
                break

    def _setup_routes(self):
        static_dir = Path(__file__).parent / "static"
        self.app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

        @self.app.get("/", response_class=HTMLResponse)
        async def root():
            index_path = static_dir / "index.html"
            return HTMLResponse(content=index_path.read_text(encoding="utf-8"))

        @self.app.get("/style.css")
        async def get_css():
            return FileResponse(static_dir / "style.css", media_type="text/css")

        @self.app.get("/dashboard.js")
        async def get_js():
            return FileResponse(static_dir / "dashboard.js", media_type="application/javascript")

        @self.app.get("/api/status")
        async def get_status():
            return {
                "system": self.config.get("system", {}).get("project_name", "OCULUS"),
                "model_mode": "PRECOMPUTED (MODULAR ARCHITECTURE)",
                "total_frames": self.streamer.total_frames,
                "is_synthetic": self.streamer.is_synthetic,
                "world_map": self.streamer.get_world_map(),
            }

        @self.app.get("/api/world_map")
        async def get_world_map():
            return self.streamer.get_world_map()

        @self.app.get("/api/frame/{frame_id}")
        async def get_frame(frame_id: int, uniform: bool = Query(False)):
            """Processes and returns telemetry, active cells, objects, and sampled points."""
            json_str = self._compute_and_cache_frame(frame_id, uniform)
            return Response(content=json_str, media_type="application/json")

    def run(self):
        """Starts uvicorn server and optionally opens the browser."""
        url = f"http://{self.host}:{self.port}"
        print(f"\n=======================================================")
        print(f"  OCULUS: Perception-Guided Adaptive 2.5D LiDAR Mapping")
        print(f"  Autonomous Vehicle Cockpit Visualizer")
        print(f"  Live URL: {url}")
        print(f"  Mode:     PRECOMPUTED (MODULAR ARCHITECTURE)")
        print(f"=======================================================\n")

        if self.auto_open:
            def open_browser():
                import time
                time.sleep(1.2)
                webbrowser.open(url)
            threading.Thread(target=open_browser, daemon=True).start()

        uvicorn.run(self.app, host=self.host, port=self.port, log_level="warning")
