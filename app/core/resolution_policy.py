"""Resolution Policy Engine for Adaptive 2.5D Mapping.
Implements the core formula:
R = f(distance, semantic importance, geometry, motion, uncertainty, path relevance)
Translates importance scores and distance zones into hierarchical grid levels.
"""

from typing import Dict, List, Optional
import numpy as np


class ResolutionPolicy:
    """Calculates optimal cell resolution based on distance zones and perception triggers."""

    # Level index to resolution in meters
    LEVEL_RESOLUTIONS = {
        0: 0.05,  # Level 0: 5 cm (Ultra Fine)
        1: 0.10,  # Level 1: 10 cm (Fine)
        2: 0.20,  # Level 2: 20 cm (Medium)
        3: 0.40,  # Level 3: 40 cm (Coarse)
        4: 0.80,  # Level 4: 80 cm (Ultra Coarse)
    }

    LEVEL_NAMES = {
        0: "5 cm (Ultra-Fine)",
        1: "10 cm (Fine)",
        2: "20 cm (Medium)",
        3: "40 cm (Coarse)",
        4: "80 cm (Ultra-Coarse)",
    }

    LEVEL_COLORS = {
        0: "#ef4444",  # Red (5 cm - Ultra-Fine)
        1: "#f59e0b",  # Amber / Orange (10 cm - Fine)
        2: "#eab308",  # Yellow (20 cm - Medium)
        3: "#0284c7",  # Light Blue (40 cm - Coarse)
        4: "#1e40af",  # Deep Blue (80 cm - Ultra-Coarse)
    }

    def __init__(
        self,
        refine_threshold_l0: float = 0.75,
        refine_threshold_l1: float = 0.55,
        refine_threshold_l2: float = 0.35,
        coarsen_threshold: float = 0.25,
    ):
        self.th_l0 = refine_threshold_l0
        self.th_l1 = refine_threshold_l1
        self.th_l2 = refine_threshold_l2
        self.th_coarsen = coarsen_threshold

    def get_base_level(self, distance: float) -> int:
        """Determines baseline distance-dependent resolution level."""
        if distance <= 10.0:
            return 0  # 5 cm
        elif distance <= 25.0:
            return 1  # 10 cm
        elif distance <= 50.0:
            return 2  # 20 cm
        elif distance <= 80.0:
            return 3  # 40 cm
        else:
            return 4  # 80 cm

    def determine_target_level(
        self,
        distance: float,
        importance_score: float,
        is_dynamic: bool = False,
        semantic_class: int = 0,
        complexity_cat: str = "LOW",
    ) -> int:
        """Computes target hierarchical resolution level [0..4] (0 is finest).
        
        Applies perception-guided refinement:
        - Pedestrians or high-importance objects in range refine by up to 2-3 levels.
        - High complexity objects refine down to finer resolution.
        - Unimportant distant background remains coarse.
        """
        base_level = self.get_base_level(distance)

        # High priority overrides
        if importance_score >= self.th_l0 or (semantic_class == 5 and distance <= 35.0):
            # Critical object (e.g. pedestrian or vehicle directly ahead)
            target = 0
        elif importance_score >= self.th_l1 or is_dynamic:
            # Medium-high priority: dynamic obstacle or high complexity
            target = min(base_level, 1)
        elif importance_score >= self.th_l2 or complexity_cat == "HIGH":
            # Moderate priority
            target = min(base_level, 2)
        else:
            # Low importance: default to distance base level
            target = base_level

        # Bounds check [0, 4]
        return int(np.clip(target, 0, 4))

    def should_refine(self, current_level: int, target_level: int) -> bool:
        """Returns True if current cell is coarser than target and should refine."""
        return current_level > target_level

    def should_coarsen(self, current_level: int, target_level: int) -> bool:
        """Returns True if current cell is finer than target and can be merged."""
        return current_level < target_level
