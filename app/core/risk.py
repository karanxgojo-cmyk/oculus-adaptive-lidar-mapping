"""Prototype Risk Estimation and Danger Assessment Module.
Evaluates spatial collision risk, danger score, and danger probability based on
distance, semantic class, motion vector, path corridor relevance, and uncertainty.
"""

from typing import Dict, Tuple
import numpy as np


class RiskEngine:
    """Computes transparent, rule-based danger metrics for spatial cells and detected objects.
    
    DISCLOSURE: This is a prototype risk assessment module based on kinematic and geometric
    corridor projections, NOT an ISO 26262 certified collision avoidance system.
    """

    def __init__(
        self,
        safe_threshold: float = 0.30,
        caution_threshold: float = 0.55,
        warning_threshold: float = 0.75,
        corridor_width: float = 3.5,
        corridor_length: float = 30.0,
    ):
        self.safe_threshold = safe_threshold
        self.caution_threshold = caution_threshold
        self.warning_threshold = warning_threshold
        self.corridor_width = corridor_width
        self.corridor_length = corridor_length

    def evaluate_risk(
        self,
        x: float,
        y: float,
        semantic_class: int,
        is_dynamic: bool,
        confidence: float,
        velocity: Tuple[float, float] = (0.0, 0.0),
    ) -> Dict[str, any]:
        """Calculates danger score, danger probability, and risk categorical level.
        
        Args:
            x, y: Coordinate relative to ego vehicle (x forward, y left)
            semantic_class: 1=Road, 3=Obstacle, 4=Vehicle, 5=Pedestrian, etc.
            is_dynamic: True if moving
            confidence: Classification certainty [0..1]
            velocity: (vx, vy) estimated velocity vector
            
        Returns:
            danger_score: float [0, 1]
            danger_probability: float [0, 1]
            level: "SAFE" | "CAUTION" | "WARNING" | "DANGER"
            in_corridor: bool
        """
        # 1. Longitudinal and Lateral proximity
        radial_dist = float(np.hypot(x, y))
        dist_factor = float(np.clip(1.0 - (radial_dist / self.corridor_length), 0.0, 1.0))

        # 2. Path corridor relevance (x > 0 and |y| <= half corridor width)
        in_front = x > 0.0
        in_lateral_corridor = abs(y) <= (self.corridor_width / 2.0)
        in_corridor = bool(in_front and in_lateral_corridor and (x <= self.corridor_length))

        # Lateral alignment score (higher the closer to the vehicle centerline y=0)
        lateral_relevance = float(np.clip(1.0 - (abs(y) / (self.corridor_width * 1.5)), 0.0, 1.0))
        path_relevance = 1.0 if in_corridor else (0.6 * lateral_relevance if in_front else 0.1)

        # 3. Semantic vulnerability / risk factor
        # Pedestrian (5) = 1.0, Vehicle (4) = 0.8, Obstacle (3) = 0.7, Wall/Pole = 0.5, Road = 0.0
        semantic_risk_table = {
            0: 0.20,  # Unknown
            1: 0.00,  # Drivable road
            2: 0.15,  # Non-drivable sidewalk/curb
            3: 0.70,  # Static obstacle
            4: 0.85,  # Vehicle
            5: 1.00,  # Vulnerable Road User (Pedestrian)
            6: 0.50,  # Pole
            7: 0.50,  # Wall
        }
        sem_risk = semantic_risk_table.get(semantic_class, 0.20)

        # 4. Motion factor
        # If dynamic and moving towards our corridor, risk surges
        vx, vy = velocity
        v_speed = float(np.hypot(vx, vy))
        motion_factor = 0.3 if not is_dynamic else 0.8
        if is_dynamic and x > 0 and (x * vx + y * vy < 0):  # closing velocity towards ego
            motion_factor = 1.0

        # 5. Uncertainty multiplier (higher uncertainty slightly elevates caution)
        uncertainty = 1.0 - confidence
        uncertainty_factor = 0.15 * uncertainty

        # 6. Combined Danger Formula
        # Danger is highest when high-risk entity (e.g. Pedestrian/Car) is close AND in forward path
        raw_danger = (
            0.40 * (dist_factor * path_relevance) +
            0.30 * sem_risk +
            0.20 * motion_factor +
            0.10 * uncertainty_factor
        )
        
        # Non-drivable road itself should remain SAFE unless an obstacle is present
        if semantic_class == 1:
            raw_danger = 0.02

        danger_score = float(np.clip(raw_danger, 0.0, 1.0))
        
        # Sigmoid calibration for danger probability
        # P(danger) = 1 / (1 + exp(-8 * (score - 0.5)))
        danger_prob = float(1.0 / (1.0 + np.exp(-10.0 * (danger_score - 0.45))))
        danger_prob = float(np.clip(danger_prob, 0.01, 0.99))

        # Categorical Level assignment
        if danger_score >= self.warning_threshold:
            level = "DANGER"
        elif danger_score >= self.caution_threshold:
            level = "WARNING"
        elif danger_score >= self.safe_threshold:
            level = "CAUTION"
        else:
            level = "SAFE"

        return {
            "danger_score": round(danger_score, 3),
            "danger_probability": round(danger_prob, 3),
            "level": level,
            "in_corridor": in_corridor,
            "path_relevance": round(path_relevance, 3),
        }
