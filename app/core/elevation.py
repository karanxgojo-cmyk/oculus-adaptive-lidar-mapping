"""Elevation fusion module for 2.5D multi-layer grid mapping.
Computes robust elevation statistics: mean, median, min, max, spread/variance.
"""

from typing import Dict, Optional
import numpy as np


class ElevationLayer:
    """Computes and updates cell elevation statistics from LiDAR point observations."""

    @staticmethod
    def compute_elevation_stats(z_values: np.ndarray) -> Dict[str, float]:
        """Calculates robust statistical summary of elevation values in a spatial bin."""
        n = len(z_values)
        if n == 0:
            return {
                "elevation": 0.0,
                "variance": 0.0,
                "min": 0.0,
                "max": 0.0,
                "spread": 0.0,
                "count": 0,
            }
        elif n == 1:
            z0 = float(z_values[0])
            return {
                "elevation": z0,
                "variance": 0.0,
                "min": z0,
                "max": z0,
                "spread": 0.0,
                "count": 1,
            }
        elif n == 2:
            z0, z1 = float(z_values[0]), float(z_values[1])
            z_mean = (z0 + z1) / 2.0
            diff = abs(z0 - z1)
            return {
                "elevation": z_mean,
                "variance": (diff * diff) / 4.0,
                "min": min(z0, z1),
                "max": max(z0, z1),
                "spread": diff,
                "count": 2,
            }

        median_z = float(np.median(z_values))
        mean_z = float(np.mean(z_values))
        var_z = float(np.var(z_values))
        min_z = float(np.min(z_values))
        max_z = float(np.max(z_values))
        spread = max_z - min_z

        return {
            "elevation": median_z,
            "variance": var_z,
            "min": min_z,
            "max": max_z,
            "spread": spread,
            "count": n,
        }

    @staticmethod
    def fuse_recursive(
        prior_elevation: float,
        prior_var: float,
        prior_count: int,
        new_elevation: float,
        new_var: float,
        new_count: int,
    ) -> Dict[str, float]:
        """Kalman / recursive weighted update for temporal elevation fusion."""
        if prior_count == 0:
            return {
                "elevation": new_elevation,
                "variance": new_var,
                "count": new_count,
            }
        
        # Weighted mean update
        total_count = prior_count + new_count
        w_prior = prior_count / total_count
        w_new = new_count / total_count
        
        fused_elev = w_prior * prior_elevation + w_new * new_elevation
        # Pooled variance approximation
        fused_var = w_prior * (prior_var + (prior_elevation - fused_elev)**2) + \
                    w_new * (new_var + (new_elevation - fused_elev)**2)

        return {
            "elevation": float(fused_elev),
            "variance": float(fused_var),
            "count": total_count,
        }
