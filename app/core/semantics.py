"""Semantic fusion and multi-class distribution tracker for 2.5D grid cells.
Maintains class probability distributions, dominant class determination, and ambiguity entropy.
"""

from typing import Dict, List, Tuple
import numpy as np


CLASS_NAMES = {
    0: "Unknown",
    1: "Drivable",
    2: "Non-Drivable",
    3: "Obstacle",
    4: "Vehicle",
    5: "Pedestrian",
    6: "Pole",
    7: "Wall",
}

CLASS_COLORS = {
    0: "#64748b",  # Gray/Slate
    1: "#10b981",  # Green
    2: "#475569",  # Dark Slate
    3: "#f59e0b",  # Amber/Yellow
    4: "#3b82f6",  # Blue
    5: "#ef4444",  # Red
    6: "#a855f7",  # Purple
    7: "#0ea5e9",  # Sky Blue
}


class SemanticLayer:
    """Manages semantic class aggregation, confidence scoring, and uncertainty quantification."""

    def __init__(self, num_classes: int = 8):
        self.num_classes = num_classes

    @staticmethod
    def aggregate_points_semantics(
        labels: np.ndarray,
        confidences: np.ndarray,
    ) -> Dict[str, any]:
        """Aggregates multiple point labels into a categorical distribution.
        
        Returns:
            dominant_class: int ID
            dominant_name: str
            confidence: float in [0, 1]
            class_probs: dict mapping class ID to probability float
            ambiguity_entropy: float in [0, 1] (0 = pure, 1 = maximum ambiguity)
        """
        n = len(labels)
        if n == 0:
            return {
                "dominant_class": 0,
                "dominant_name": CLASS_NAMES[0],
                "confidence": 0.0,
                "class_probs": {c: (1.0 if c == 0 else 0.0) for c in range(8)},
                "ambiguity_entropy": 0.0,
            }
        elif n == 1:
            lbl_id = int(labels[0]) % 8
            conf = float(confidences[0]) if confidences is not None else 1.0
            return {
                "dominant_class": lbl_id,
                "dominant_name": CLASS_NAMES.get(lbl_id, "Unknown"),
                "confidence": conf,
                "class_probs": {c: (conf if c == lbl_id else 0.0) for c in range(8)},
                "ambiguity_entropy": 0.0,
            }

        # Weighted count using point confidences
        weights = confidences if confidences is not None else np.ones(len(labels), dtype=np.float32)
        total_weight = float(np.sum(weights))
        if total_weight <= 0:
            total_weight = 1.0

        class_weights = np.zeros(8, dtype=np.float32)
        for lbl, w in zip(labels, weights):
            lbl_id = int(lbl) % 8
            class_weights[lbl_id] += float(w)

        probs = class_weights / total_weight
        dom_class = int(np.argmax(probs))
        dom_conf = float(probs[dom_class])

        # Compute normalized Shannon entropy as ambiguity measure
        # H = -sum(p * log2(p)) / log2(K)
        non_zero = probs[probs > 1e-6]
        if len(non_zero) > 1:
            entropy = -np.sum(non_zero * np.log2(non_zero))
            max_entropy = np.log2(len(probs))
            norm_entropy = float(entropy / max_entropy)
        else:
            norm_entropy = 0.0

        return {
            "dominant_class": dom_class,
            "dominant_name": CLASS_NAMES.get(dom_class, "Unknown"),
            "confidence": dom_conf,
            "class_probs": {c: float(probs[c]) for c in range(8)},
            "ambiguity_entropy": norm_entropy,
        }

    @staticmethod
    def fuse_with_prior(
        prior_probs: Dict[int, float],
        prior_weight: float,
        new_probs: Dict[int, float],
        new_weight: float,
    ) -> Dict[str, any]:
        """Temporal Bayesian blend of prior probability distribution with new frame distribution."""
        total_w = prior_weight + new_weight
        if total_w <= 0:
            total_w = 1.0

        blended_probs = np.zeros(8, dtype=np.float32)
        for c in range(8):
            p_prior = prior_probs.get(c, 0.0)
            p_new = new_probs.get(c, 0.0)
            blended_probs[c] = (prior_weight * p_prior + new_weight * p_new) / total_w

        dom_class = int(np.argmax(blended_probs))
        dom_conf = float(blended_probs[dom_class])

        non_zero = blended_probs[blended_probs > 1e-6]
        if len(non_zero) > 1:
            entropy = -np.sum(non_zero * np.log2(non_zero))
            norm_entropy = float(entropy / np.log2(8))
        else:
            norm_entropy = 0.0

        return {
            "dominant_class": dom_class,
            "dominant_name": CLASS_NAMES.get(dom_class, "Unknown"),
            "confidence": dom_conf,
            "class_probs": {c: float(blended_probs[c]) for c in range(8)},
            "ambiguity_entropy": norm_entropy,
        }
