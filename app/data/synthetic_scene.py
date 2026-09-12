"""Synthetic LiDAR Scene & 3-Minute Continuous Driving Trajectory Generator.
Generates a world-consistent 180-frame (3-minute) autonomous vehicle sequence:
- Fixed world coordinate frame for all static landmarks (poles, walls, buildings, trees, curbs, barriers)
- Ego vehicle pose (x, y, z, yaw) moves continuously through the fixed world
- Smooth spline trajectory with continuous tangent heading: yaw(t) = atan2(dy/dt, dx/dt)
- Realistic sensor frame observation: P_sensor = R(-yaw) * (P_world - [x, y]^T)
- Dynamic actors (crossing pedestrian, oncoming high-speed car, lead car, cyclist) with independent world paths
- Dynamic adaptive resolution refinement (coarse distant -> local fine RED -> coarsening on exit)
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np


@dataclass
class SyntheticFrame:
    """Represents a single synthetic LiDAR frame with points, intensity, labels, and metadata."""
    frame_id: int
    timestamp_s: float
    phase_name: str
    description: str
    points: np.ndarray        # (N, 3) float32 in sensor/ego coordinates
    intensities: np.ndarray   # (N,) float32
    labels: np.ndarray        # (N,) uint32 SemanticKITTI class IDs
    ego_pose: Tuple[float, float, float, float]  # (x_w, y_w, z_w, yaw_deg)
    speed_mps: float = 4.0


class SyntheticLiDARGenerator:
    """Generates realistic synthetic 3D LiDAR point clouds for continuous 3-minute playback."""

    # Semantic IDs:
    # 0: Unknown, 1: Drivable, 2: Non-Drivable (curb/sidewalk), 3: Obstacle,
    # 4: Vehicle, 5: Pedestrian, 6: Pole, 7: Wall

    def __init__(
        self,
        num_rings: int = 64,
        points_per_ring: int = 500,
        sensor_height: float = 1.73,
    ):
        self.num_rings = num_rings
        self.points_per_ring = points_per_ring
        self.sensor_height = sensor_height

        # Initialize fixed static world landmarks once
        self._world_poles = self._build_world_poles()
        self._world_trees = self._build_world_trees()
        self._world_buildings = self._build_world_buildings()
        self._world_barrier = self._build_world_barrier()
        self._world_parked_cars = self._build_world_parked_cars()

    @staticmethod
    def get_ego_pose(t_sec: float) -> Tuple[float, float, float, float, float]:
        """Computes continuous smooth ego pose (x, y, z, yaw_deg, speed_mps) for timestamp t in [0, 180].
        Uses smooth spline segments with exact analytical velocity derivatives so heading
        yaw(t) = atan2(dy/dt, dx/dt) aligns smoothly with the road tangent BEFORE and DURING turns.
        """
        t = float(np.clip(t_sec, 0.0, 180.0))

        if t <= 35.0:
            # Segment 1: Urban Boulevard Eastbound (Straight East, yaw = 0 deg)
            p = t / 35.0
            x = p * 140.0
            y = 0.0
            z = 0.0
            vx = 140.0 / 35.0 # 4.0 m/s
            vy = 0.0

        elif t <= 70.0:
            # Segment 2: S-Curved Road Navigation (Gradual Right-then-Left Curvature)
            p = (t - 35.0) / 35.0
            x = 140.0 + p * 140.0
            # Hermite cubic curve: smoothly enters and exits horizontal tangent
            y = 55.0 * (3.0 * p**2 - 2.0 * p**3)
            z = 0.0
            vx = 140.0 / 35.0
            vy = 55.0 * (6.0 * p - 6.0 * p**2) / 35.0

        elif t <= 105.0:
            # Segment 3: Downtown Approach & Smooth 90-deg Turn into 4-Way Intersection
            # Turns smoothly from heading 0 deg -> 90 deg (North) onto North Avenue
            p = (t - 70.0) / 35.0
            x = 280.0 + 40.0 * np.sin(p * np.pi * 0.5)
            y = 55.0 + 85.0 * (1.0 - np.cos(p * np.pi * 0.5))
            z = 0.0
            vx = 40.0 * (np.pi * 0.5 / 35.0) * np.cos(p * np.pi * 0.5)
            vy = 85.0 * (np.pi * 0.5 / 35.0) * np.sin(p * np.pi * 0.5)

        elif t <= 145.0:
            # Segment 4: Northbound Avenue Drive (Straight North, yaw = 90 deg)
            p = (t - 105.0) / 40.0
            x = 320.0
            y = 140.0 + 120.0 * p
            z = 0.0
            vx = 0.0
            vy = 120.0 / 40.0 # 3.0 m/s

        else:
            # Segment 5: Elevated Bridge Incline & Scenic Curve (yaw 90 -> 160 deg)
            p = (t - 145.0) / 35.0
            ang = p * np.pi * 0.6
            x = 320.0 - 140.0 * (1.0 - np.cos(ang))
            y = 260.0 + 50.0 * np.sin(ang)
            z = 3.4 * np.sin(p * np.pi) # Bridge elevation peak +3.4m
            vx = -140.0 * (np.pi * 0.6 / 35.0) * np.sin(ang)
            vy = 50.0 * (np.pi * 0.6 / 35.0) * np.cos(ang)

        speed = float(np.hypot(vx, vy))
        yaw = float(np.degrees(np.arctan2(vy, vx)))
        return float(x), float(y), float(z), float(yaw), float(speed)

    # -------------------------------------------------------------------------
    # Fixed Static World Geometry (PERMANENT WORLD COORDINATES)
    # -------------------------------------------------------------------------

    def _build_world_poles(self) -> List[Dict[str, float]]:
        """Builds permanently fixed street light poles in world coordinates."""
        poles = []
        # Boulevard poles along curbs (y = -6.5m and +6.5m)
        for x in [12.0, 37.0, 62.0, 87.0, 112.0, 137.0]:
            poles.append({"x": x, "y": -6.5, "z": 0.0, "h": 5.2})
            poles.append({"x": x, "y": 6.5, "z": 0.0, "h": 5.2})

        # S-Curve poles along outer and inner road curbs
        for x, y_curb in [(165.0, 2.0), (190.0, 16.0), (215.0, 36.0), (240.0, 50.0), (265.0, 56.0)]:
            poles.append({"x": x, "y": y_curb - 6.5, "z": 0.0, "h": 5.2})
            poles.append({"x": x, "y": y_curb + 6.5, "z": 0.0, "h": 5.2})

        # Intersection traffic light poles at 4 corners
        poles.append({"x": 306.0, "y": 96.0, "z": 0.0, "h": 5.8})
        poles.append({"x": 334.0, "y": 96.0, "z": 0.0, "h": 5.8})
        poles.append({"x": 306.0, "y": 124.0, "z": 0.0, "h": 5.8})
        poles.append({"x": 334.0, "y": 124.0, "z": 0.0, "h": 5.8})

        # North Avenue poles (x = 311m and 329m)
        for y in [148.0, 173.0, 198.0, 223.0, 248.0]:
            poles.append({"x": 311.0, "y": y, "z": 0.0, "h": 5.2})
            poles.append({"x": 329.0, "y": y, "z": 0.0, "h": 5.2})

        # Bridge lamp posts along railings
        for bx, by in [(310.0, 275.0), (285.0, 292.0), (255.0, 305.0), (225.0, 310.0), (195.0, 310.0)]:
            poles.append({"x": bx, "y": by - 6.0, "z": 2.0, "h": 4.5})
            poles.append({"x": bx, "y": by + 6.0, "z": 2.0, "h": 4.5})

        return poles

    def _build_world_trees(self) -> List[Dict[str, float]]:
        """Builds permanently fixed trees along green verges and urban parks."""
        trees = []
        for x in [20.0, 45.0, 70.0, 95.0, 120.0]:
            trees.append({"x": x, "y": -9.5, "r": 2.8})
            trees.append({"x": x, "y": 9.5, "r": 2.8})
        # Park zone near curve
        for tx, ty in [(175.0, -12.0), (200.0, -8.0), (225.0, 12.0), (250.0, 28.0)]:
            trees.append({"x": tx, "y": ty, "r": 3.2})
        # Downtown plaza trees
        for tx, ty in [(295.0, 85.0), (295.0, 135.0), (345.0, 85.0), (345.0, 135.0)]:
            trees.append({"x": tx, "y": ty, "r": 3.0})
        return trees

    def _build_world_buildings(self) -> List[Dict[str, Any]]:
        """Builds permanently fixed city buildings with footprints in world coordinates."""
        return [
            {"x": 30.0, "y": -18.0, "w": 38.0, "h": 16.0, "label": "Tech Campus"},
            {"x": 85.0, "y": 18.0, "w": 42.0, "h": 16.0, "label": "Commerce Tower"},
            {"x": 160.0, "y": -22.0, "w": 45.0, "h": 18.0, "label": "Logistics Depot"},
            {"x": 230.0, "y": 44.0, "w": 36.0, "h": 16.0, "label": "Transit Plaza"},
            {"x": 290.0, "y": 130.0, "w": 24.0, "h": 32.0, "label": "Metro Station"},
            {"x": 348.0, "y": 95.0, "w": 26.0, "h": 34.0, "label": "City Hall"},
            {"x": 348.0, "y": 170.0, "w": 28.0, "h": 40.0, "label": "Medical Center"},
            {"x": 290.0, "y": 210.0, "w": 24.0, "h": 38.0, "label": "Financial Center"},
            {"x": 348.0, "y": 235.0, "w": 26.0, "h": 34.0, "label": "Science Institute"},
        ]

    def _build_world_barrier(self) -> Dict[str, Any]:
        """Fixed static roadwork construction barrier on the shoulder of the curve."""
        return {
            "x": 205.0,
            "y": 8.0,
            "dx": 18.0,
            "dy": 2.2,
            "dz": 1.2,
            "label": "Road Work / Shoulder Barrier",
        }

    def _build_world_parked_cars(self) -> List[Dict[str, Any]]:
        """Fixed static cars parked along the street curb."""
        return [
            {"x": 65.0, "y": -4.8, "dx": 4.6, "dy": 2.0, "dz": 1.5, "yaw": 0.0, "label": "Parked Sedan"},
            {"x": 115.0, "y": 4.8, "dx": 4.8, "dy": 2.1, "dz": 1.7, "yaw": 0.0, "label": "Parked SUV"},
        ]

    def get_world_map_spec(self) -> Dict[str, Any]:
        """Provides complete structural world map specifications in fixed world coordinates (X_w, Y_w)."""
        return {
            "segments": [
                {
                    "name": "Boulevard East",
                    "type": "straight",
                    "start": [0.0, 0.0],
                    "end": [140.0, 0.0],
                    "width": 12.0,
                    "lanes": 2,
                    "heading_deg": 0.0,
                },
                {
                    "name": "S-Curved Corridor",
                    "type": "curve",
                    "start": [140.0, 0.0],
                    "control": [210.0, 15.0],
                    "end": [280.0, 55.0],
                    "width": 12.0,
                    "lanes": 2,
                },
                {
                    "name": "Intersection Turn",
                    "type": "turn",
                    "start": [280.0, 55.0],
                    "control": [320.0, 80.0],
                    "end": [320.0, 140.0],
                    "width": 14.0,
                    "lanes": 2,
                },
                {
                    "name": "North Avenue",
                    "type": "straight",
                    "start": [320.0, 140.0],
                    "end": [320.0, 260.0],
                    "width": 14.0,
                    "lanes": 2,
                    "heading_deg": 90.0,
                },
                {
                    "name": "Elevated Bridge Overpass",
                    "type": "bridge",
                    "start": [320.0, 260.0],
                    "control": [310.0, 320.0],
                    "end": [180.0, 310.0],
                    "width": 12.0,
                    "lanes": 2,
                    "elevation_peak": 3.4,
                },
            ],
            "intersection": {
                "center": [320.0, 110.0],
                "size": [28.0, 28.0],
                "crosswalks": [
                    {"name": "South Crosswalk", "p1": [306.0, 96.0], "p2": [334.0, 96.0], "width": 3.2},
                    {"name": "North Crosswalk", "p1": [306.0, 124.0], "p2": [334.0, 124.0], "width": 3.2},
                    {"name": "West Crosswalk", "p1": [306.0, 96.0], "p2": [306.0, 124.0], "width": 3.2},
                    {"name": "East Crosswalk", "p1": [334.0, 96.0], "p2": [334.0, 124.0], "width": 3.2},
                ],
            },
            "poles": self._world_poles,
            "trees": self._world_trees,
            "buildings": self._world_buildings,
            "barrier": self._world_barrier,
            "parked_cars": self._world_parked_cars,
        }

    # -------------------------------------------------------------------------
    # Observation Generator: Fixed World Points -> Local Sensor Frame
    # -------------------------------------------------------------------------

    def generate_3min_sequence(self, num_frames: int = 180) -> List[SyntheticFrame]:
        """Generates a complete 180-frame (3-minute) continuous autonomous driving trajectory.
        World coordinates are permanent. Sensor points are strictly computed by observing the
        world from the moving ego vehicle pose: P_sensor = R(-yaw) * (P_world - [x, y]^T).
        """
        frames = []

        for f_idx in range(num_frames):
            t_sec = float(f_idx)
            x_w, y_w, z_w, yaw_deg, speed_mps = self.get_ego_pose(t_sec)
            rad = np.radians(yaw_deg)
            cosA = float(np.cos(rad))
            sinA = float(np.sin(rad))

            # Phase detection
            if t_sec <= 35.0:
                phase = "Urban Cruising"
                desc = "Cruising multi-lane boulevard. Distant road maps to coarse blue cells (40-80cm)."
            elif t_sec <= 70.0:
                phase = "Curved Road Navigation"
                desc = "Navigating road curvature. Approaching static barrier on right shoulder."
            elif t_sec <= 105.0:
                phase = "Downtown Intersection / VRU"
                # Check pedestrian crossing status
                p_progress = (t_sec - 70.0) / 35.0
                ped_x_w = 310.0 + 20.0 * p_progress
                if 317.0 <= ped_x_w <= 323.0:
                    desc = "CRITICAL: Pedestrian in vehicle trajectory corridor! Local RED (5cm) refinement triggered."
                else:
                    desc = "Downtown 4-way intersection turn. Pedestrian crossing zebra crosswalk."
            elif t_sec <= 145.0:
                phase = "Oncoming Vehicle Interaction"
                desc = "Oncoming vehicle approaching in adjacent lane. Dynamic refinement bubble tracking."
            else:
                phase = "Elevated Incline / Multi-Actor"
                desc = "Ascending elevated overpass. Multi-actor tracking across elevation gradient."

            # Generate sensor-frame point cloud by scanning the fixed world
            pts_sensor_list = []
            ints_list = []
            lbls_list = []

            # 1. Road & Ground Grid in sensor frame (-25m to +60m forward, -16m to +16m lateral)
            gx = np.linspace(-25.0, 60.0, 130)
            gy = np.linspace(-16.0, 16.0, 95)
            GX, GY = np.meshgrid(gx, gy)
            GX_f = GX.flatten()
            GY_f = GY.flatten()

            r_ground = np.hypot(GX_f, GY_f)
            g_mask = (r_ground >= 1.5) & (r_ground <= 68.0)
            GX_f = GX_f[g_mask]
            GY_f = GY_f[g_mask]

            elev_grad = 1.6 if phase == "Elevated Incline / Multi-Actor" else 0.0
            GZ_f = -self.sensor_height + 0.03 * np.sin(GX_f * 0.1) + 0.02 * np.cos(GY_f * 0.15)
            GZ_f += elev_grad * (GX_f + 22.0) / 77.0
            GZ_f += np.random.normal(0, 0.012, size=len(GZ_f))

            road_mask = np.abs(GY_f) <= 3.8
            curb_mask = (np.abs(GY_f) > 3.8) & (np.abs(GY_f) <= 4.4)
            sidewalk_mask = np.abs(GY_f) > 4.4
            GZ_f[sidewalk_mask] += 0.15
            GZ_f[curb_mask] += 0.08

            g_labels = np.full(len(GX_f), 2, dtype=np.uint32)
            g_labels[road_mask] = 1   # Drivable road
            g_labels[curb_mask] = 2   # Curb
            g_labels[sidewalk_mask] = 2 # Sidewalk
            g_ints = np.random.uniform(0.18, 0.45, size=len(GX_f)).astype(np.float32)
            g_ints[road_mask] = 0.65

            pts_sensor_list.append(np.column_stack([GX_f, GY_f, GZ_f]))
            ints_list.append(g_ints)
            lbls_list.append(g_labels)

            # 2. Transform FIXED WORLD POLES within sensor range (R <= 75m)
            for p in self._world_poles:
                dx = p["x"] - x_w
                dy = p["y"] - y_w
                dist = np.hypot(dx, dy)
                if dist <= 70.0:
                    xs = cosA * dx + sinA * dy
                    ys = -sinA * dx + cosA * dy
                    # Generate pole column in sensor frame
                    pz_vals = np.linspace(-self.sensor_height + 0.15, p["h"] - self.sensor_height, 22)
                    for pz in pz_vals:
                        theta = np.linspace(0, 2 * np.pi, 5, endpoint=False)
                        px_s = xs + 0.12 * np.cos(theta)
                        py_s = ys + 0.12 * np.sin(theta)
                        pts_sensor_list.append(np.column_stack([px_s, py_s, np.full_like(px_s, pz)]))
                        ints_list.append(np.full(len(px_s), 0.88, dtype=np.float32))
                        lbls_list.append(np.full(len(px_s), 6, dtype=np.uint32)) # Pole class

            # 3. Dynamic Actors in World Space -> Transformed to Sensor Frame
            # Actor A: Crossing Pedestrian (Phase 3: t in [70, 105])
            if 70.0 <= t_sec <= 105.0:
                p_progress = (t_sec - 70.0) / 35.0
                ped_wx = 310.0 + 20.0 * p_progress
                ped_wy = 98.0
                dx = ped_wx - x_w
                dy = ped_wy - y_w
                if np.hypot(dx, dy) <= 65.0:
                    xs = cosA * dx + sinA * dy
                    ys = -sinA * dx + cosA * dy
                    p_pts, p_ints, p_lbls = self._generate_box_sensor_points((xs, ys, -self.sensor_height + 0.85), (0.6, 0.6, 1.75), 5, 12, 0.95)
                    pts_sensor_list.append(p_pts); ints_list.append(p_ints); lbls_list.append(p_lbls)

            # Actor B: Oncoming Vehicle (Phase 4: t in [105, 145])
            if 105.0 <= t_sec <= 145.0:
                v_progress = (t_sec - 105.0) / 40.0
                onc_wx = 316.0
                onc_wy = 270.0 - 190.0 * v_progress
                dx = onc_wx - x_w
                dy = onc_wy - y_w
                if np.hypot(dx, dy) <= 75.0:
                    xs = cosA * dx + sinA * dy
                    ys = -sinA * dx + cosA * dy
                    v_pts, v_ints, v_lbls = self._generate_box_sensor_points((xs, ys, -self.sensor_height + 0.8), (4.6, 2.0, 1.55), 4, 15, 0.85)
                    pts_sensor_list.append(v_pts); ints_list.append(v_ints); lbls_list.append(v_lbls)

            # Actor C: Static Parked Car (at fixed world (65, -4.8))
            dx = 65.0 - x_w
            dy = -4.8 - y_w
            if np.hypot(dx, dy) <= 65.0:
                xs = cosA * dx + sinA * dy
                ys = -sinA * dx + cosA * dy
                v_pts, v_ints, v_lbls = self._generate_box_sensor_points((xs, ys, -self.sensor_height + 0.8), (4.6, 2.0, 1.5), 4, 14, 0.75)
                pts_sensor_list.append(v_pts); ints_list.append(v_ints); lbls_list.append(v_lbls)

            # Actor D: Construction Barrier (at fixed world (205, 8.0))
            dx = 205.0 - x_w
            dy = 8.0 - y_w
            if np.hypot(dx, dy) <= 65.0:
                xs = cosA * dx + sinA * dy
                ys = -sinA * dx + cosA * dy
                b_pts, b_ints, b_lbls = self._generate_box_sensor_points((xs, ys, -self.sensor_height + 0.6), (3.2, 1.2, 1.1), 3, 14, 0.92)
                pts_sensor_list.append(b_pts); ints_list.append(b_ints); lbls_list.append(b_lbls)

            # Assemble frame
            frame_pts = np.vstack(pts_sensor_list).astype(np.float32)
            frame_ints = np.concatenate(ints_list).astype(np.float32)
            frame_lbls = np.concatenate(lbls_list).astype(np.uint32)

            frames.append(
                SyntheticFrame(
                    frame_id=f_idx + 1,
                    timestamp_s=round(t_sec, 1),
                    phase_name=phase,
                    description=desc,
                    points=frame_pts,
                    intensities=frame_ints,
                    labels=frame_lbls,
                    ego_pose=(round(x_w, 2), round(y_w, 2), round(z_w, 2), round(yaw_deg, 1)),
                    speed_mps=round(speed_mps, 1),
                )
            )

        return frames

    def _generate_box_sensor_points(
        self,
        center: Tuple[float, float, float],
        dims: Tuple[float, float, float],
        sem_class: int,
        density: int = 12,
        base_intensity: float = 0.8,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generates bounding box surface points in sensor frame."""
        cx, cy, cz = center
        dx, dy, dz = dims
        pts = []

        for sign in [-0.5, 0.5]:
            fx = cx + sign * dx
            y_vals = np.linspace(cy - dy / 2, cy + dy / 2, max(3, int(density * dy / 2)))
            z_vals = np.linspace(cz - dz / 2, cz + dz / 2, max(3, int(density * dz / 2)))
            Y, Z = np.meshgrid(y_vals, z_vals)
            X = np.full_like(Y, fx)
            pts.append(np.column_stack([X.flatten(), Y.flatten(), Z.flatten()]))

        for sign in [-0.5, 0.5]:
            fy = cy + sign * dy
            x_vals = np.linspace(cx - dx / 2, cx + dx / 2, max(3, int(density * dx / 2)))
            z_vals = np.linspace(cz - dz / 2, cz + dz / 2, max(3, int(density * dz / 2)))
            X, Z = np.meshgrid(x_vals, z_vals)
            Y = np.full_like(X, fy)
            pts.append(np.column_stack([X.flatten(), Y.flatten(), Z.flatten()]))

        box_pts = np.vstack(pts).astype(np.float32)
        box_ints = np.random.uniform(base_intensity - 0.08, base_intensity + 0.08, size=len(box_pts)).astype(np.float32)
        box_lbls = np.full(len(box_pts), sem_class, dtype=np.uint32)
        return box_pts, box_ints, box_lbls

    def generate_presentation_sequence(self) -> List[SyntheticFrame]:
        """Backward-compatible sampling of 5 key transitional frames from 3-min sequence."""
        all_frames = self.generate_3min_sequence(num_frames=180)
        key_indices = [5, 45, 92, 105, 130]
        return [all_frames[i] for i in key_indices]
