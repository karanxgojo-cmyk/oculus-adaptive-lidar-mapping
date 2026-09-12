"""Real-time Telemetry & Benchmark Metrics Tracker.
Computes genuine measured latency, FPS, point count, active adaptive cells,
theoretical baseline uniform cells, and estimated cell storage reduction.
"""

from dataclasses import dataclass
import time
from typing import Dict, List


@dataclass
class SystemMetrics:
    """Live performance metrics container with strict technical honesty labeling."""
    fps: float                                 # Measured frames per second
    latency_ms: float                          # Measured total pipeline latency in ms
    point_count: int                           # Measured active LiDAR points
    active_adaptive_cells: int                 # Measured active leaf cells in hierarchical grid
    theoretical_uniform_cells: int             # Calculated uniform 5cm cells for same bounding area
    cell_reduction_percent: float              # Calculated reduction percentage
    estimated_adaptive_storage_kb: float       # Estimated cell storage (based on 64 bytes/cell)
    estimated_uniform_storage_kb: float        # Estimated uniform cell storage (64 bytes/cell)
    storage_reduction_percent: float           # Estimated storage saving percentage
    current_frame: int
    dynamic_object_count: int
    perception_mode: str = "DEMO/SIMULATED"    # Honest label: REAL vs DEMO
    metrics_label: str = "MEASURED & ESTIMATED"


class MetricsTracker:
    """Tracks frame timings and cell storage benchmarks."""

    # Approximate byte footprint per active 2.5D cell struct in memory
    BYTES_PER_CELL = 64

    def __init__(self, history_len: int = 15):
        self.history_len = history_len
        self.frame_times: List[float] = []
        self.frame_count = 0

    def record_frame(
        self,
        latency_ms: float,
        point_count: int,
        adaptive_cells: int,
        uniform_cells: int,
        dynamic_count: int,
        perception_mode: str = "DEMO/SIMULATED",
    ) -> SystemMetrics:
        """Records a completed frame and calculates rolling FPS and reduction ratios."""
        self.frame_count += 1
        now = time.perf_counter()
        self.frame_times.append(now)
        if len(self.frame_times) > self.history_len:
            self.frame_times.pop(0)

        # Compute genuine rolling FPS
        if len(self.frame_times) > 1:
            duration = self.frame_times[-1] - self.frame_times[0]
            fps = float((len(self.frame_times) - 1) / duration) if duration > 0 else 0.0
        else:
            fps = float(1000.0 / max(latency_ms, 1.0))

        # Cell and Storage reduction calculation
        if uniform_cells > 0:
            reduction_pct = ((uniform_cells - adaptive_cells) / uniform_cells) * 100.0
            reduction_pct = max(0.0, min(reduction_pct, 99.9))
        else:
            reduction_pct = 0.0

        adaptive_kb = (adaptive_cells * self.BYTES_PER_CELL) / 1024.0
        uniform_kb = (uniform_cells * self.BYTES_PER_CELL) / 1024.0

        return SystemMetrics(
            fps=round(fps, 1),
            latency_ms=round(latency_ms, 2),
            point_count=point_count,
            active_adaptive_cells=adaptive_cells,
            theoretical_uniform_cells=uniform_cells,
            cell_reduction_percent=round(reduction_pct, 1),
            estimated_adaptive_storage_kb=round(adaptive_kb, 1),
            estimated_uniform_storage_kb=round(uniform_kb, 1),
            storage_reduction_percent=round(reduction_pct, 1),
            current_frame=self.frame_count,
            dynamic_object_count=dynamic_count,
            perception_mode=perception_mode,
        )
