"""Map rendering helper utilities.
Generates color scales, coordinates projections, and canvas drawing payloads.
"""

from typing import Dict, List, Tuple
import numpy as np


class MapRenderer:
    """Provides color mapping and coordinate transformations for 2D/2.5D map visualization."""

    # Level colors in RGBA: RED = FINE, YELLOW = MEDIUM, BLUE = COARSE
    LEVEL_COLORS = {
        0: "rgba(239, 68, 68, 0.60)",   # 5 cm Red (Fine)
        1: "rgba(245, 158, 11, 0.50)",  # 10 cm Amber / Orange
        2: "rgba(234, 179, 8, 0.45)",   # 20 cm Yellow (Medium)
        3: "rgba(2, 132, 199, 0.35)",   # 40 cm Light Blue (Coarse)
        4: "rgba(30, 64, 175, 0.25)",   # 80 cm Deep Blue (Ultra-Coarse)
    }

    # Semantic Colors Hex
    SEMANTIC_COLORS = {
        0: "#64748b",  # Unknown
        1: "#10b981",  # Drivable Road (Green)
        2: "#475569",  # Non-Drivable Sidewalk (Dark Gray)
        3: "#f59e0b",  # Static Obstacle (Amber)
        4: "#3b82f6",  # Vehicle (Blue)
        5: "#ef4444",  # Pedestrian (Red)
        6: "#a855f7",  # Pole (Purple)
        7: "#0ea5e9",  # Wall (Cyan-Blue)
    }

    # Danger Colors
    DANGER_COLORS = {
        "SAFE": "#10b981",
        "CAUTION": "#f59e0b",
        "WARNING": "#f97316",
        "DANGER": "#ef4444",
    }

    @staticmethod
    def elevation_to_color(elevation: float, min_e: float = -2.5, max_e: float = 2.0) -> str:
        """Maps elevation height (z) to a turbo/plasma color gradient string."""
        norm = np.clip((elevation - min_e) / max(max_e - min_e, 0.1), 0.0, 1.0)
        # Blue (low) -> Green (ground) -> Yellow -> Red (high)
        if norm < 0.33:
            r = 0
            g = int(norm * 3.0 * 255)
            b = 255
        elif norm < 0.66:
            r = int((norm - 0.33) * 3.0 * 255)
            g = 255
            b = int((1.0 - (norm - 0.33) * 3.0) * 255)
        else:
            r = 255
            g = int((1.0 - (norm - 0.66) * 3.0) * 255)
            b = 0
        return f"rgb({r},{g},{b})"
