"""Perception-Guided Importance and Geometric Complexity Engine.
Calculates transparent spatial priority scores from distance, semantics, motion,
surface geometry, and path relevance to guide hierarchical grid refinement.
"""

from typing import Dict, Optional, Tuple
import numpy as np


class ImportanceEngine:
    """Computes transparent importance scores to drive adaptive grid resolution policy.
    
    DISCLOSURE: This is a rule-based perception-guided resolution policy module,
    NOT a trained black-box neural network.
    """

    def __init__(self, config_weights: Optional[Dict[str, float]] = None):
        weights = config_weights or {}
        self.w_dist = weights.get("distance", 0.25)
        self.w_sem = weights.get("semantic", 0.30)
        self.w_mot = weights.get("motion", 0.20)
        self.w_unc = weights.get("uncertainty", 0.10)
        self.w_geom = weights.get("geometry", 0.15)
        self.w_path = weights.get("path_relevance", 0.25)
        
        # Normalize weights so they sum to 1.0 (excluding path which is an additive booster)
        base_sum = self.w_dist + self.w_sem + self.w_mot + self.w_unc + self.w_geom
        if base_sum > 0:
            self.w_dist /= base_sum
            self.w_sem /= base_sum
            self.w_mot /= base_sum
            self.w_unc /= base_sum
            self.w_geom /= base_sum

    @staticmethod
    def compute_geometric_complexity(points_in_cell: np.ndarray) -> Tuple[float, str]:
        """Calculates geometric complexity score [0, 1] and category (LOW/MEDIUM/HIGH).
        
        Features:
        - Local point density (normalized up to 100 points)
        - Height spread (delta z)
        - Surface roughness using smallest eigenvalue ratio of 3D covariance matrix
        """
        if len(points_in_cell) < 3:
            return 0.0, "LOW"

        # 1. Density score
        density_score = min(len(points_in_cell) / 80.0, 1.0)

        # 2. Height variation (delta z)
        z_vals = points_in_cell[:, 2]
        delta_z = float(np.max(z_vals) - np.min(z_vals))
        height_score = min(delta_z / 2.0, 1.0)

        # Fast path for flat ground surfaces
        if delta_z < 0.10:
            raw_complexity = 0.25 * density_score + 0.35 * height_score
            return round(float(raw_complexity), 3), "LOW"

        # 3. Surface roughness via covariance eigenvalues
        centered = points_in_cell - np.mean(points_in_cell, axis=0)
        cov = (centered.T @ centered) / len(points_in_cell)
        try:
            eigvals = np.linalg.eigvalsh(cov)
            eigvals = np.sort(np.maximum(eigvals, 0.0))
            sum_eigs = np.sum(eigvals)
            # Surface variation / curvature: lambda_0 / (lambda_0 + lambda_1 + lambda_2)
            roughness = float(eigvals[0] / sum_eigs) if sum_eigs > 1e-6 else 0.0
            roughness_score = min(roughness * 3.0, 1.0)  # scaled
        except Exception:
            roughness_score = 0.0

        # Weighted combination
        raw_complexity = 0.35 * height_score + 0.40 * roughness_score + 0.25 * density_score
        complexity_score = float(np.clip(raw_complexity, 0.0, 1.0))

        if complexity_score >= 0.65:
            cat = "HIGH"
        elif complexity_score >= 0.35:
            cat = "MEDIUM"
        else:
            cat = "LOW"

        return round(complexity_score, 3), cat

    def compute_importance(
        self,
        distance: float,
        semantic_class: int,
        is_dynamic: bool,
        confidence: float,
        geometry_score: float,
        path_relevance: float,
        max_range: float = 80.0,
    ) -> Dict[str, float]:
        """Calculates multi-criteria importance score in range [0, 1]."""
        # Distance score: closer = higher score
        dist_score = float(np.clip(1.0 - (distance / max_range), 0.0, 1.0))

        # Semantic score: Pedestrian (5) > Vehicle (4) > Obstacle (3) > Pole/Wall > Road
        semantic_priors = {
            0: 0.10,  # Unknown
            1: 0.05,  # Drivable road (flat background)
            2: 0.20,  # Sidewalk
            3: 0.70,  # Static obstacle
            4: 0.85,  # Vehicle
            5: 0.98,  # Pedestrian
            6: 0.40,  # Pole
            7: 0.35,  # Wall
        }
        sem_score = semantic_priors.get(semantic_class, 0.10)

        # Motion score
        mot_score = 1.0 if is_dynamic else 0.0

        # Uncertainty score: lower confidence -> higher refinement need
        unc_score = float(np.clip(1.0 - confidence, 0.0, 1.0))

        # Base composite score
        base_importance = (
            self.w_dist * dist_score +
            self.w_sem * sem_score +
            self.w_mot * mot_score +
            self.w_unc * unc_score +
            self.w_geom * geometry_score
        )

        # Path relevance booster: objects in forward trajectory get direct prioritization
        boosted = base_importance + (self.w_path * path_relevance * (0.5 + 0.5 * sem_score))
        final_importance = float(np.clip(boosted, 0.0, 1.0))

        return {
            "importance_score": round(final_importance, 3),
            "distance_score": round(dist_score, 3),
            "semantic_score": round(sem_score, 3),
            "motion_score": round(mot_score, 3),
            "uncertainty_score": round(unc_score, 3),
            "geometry_score": round(geometry_score, 3),
            "path_relevance_score": round(path_relevance, 3),
        }
