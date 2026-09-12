"""Hierarchical Adaptive 2.5D LiDAR Grid Mapping.
Implements multi-resolution quadtree grid storage, seamless parent-child relationships,
elevation fusion, semantic aggregation, occupancy updating, and dynamic refinement/merging.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple
import numpy as np
import time

from app.core.elevation import ElevationLayer
from app.core.semantics import SemanticLayer, CLASS_NAMES, CLASS_COLORS
from app.core.resolution_policy import ResolutionPolicy


@dataclass
class GridCell:
    """Represents an individual 2.5D cell in the hierarchical grid."""
    level: int                       # 0 (5cm), 1 (10cm), 2 (20cm), 3 (40cm), 4 (80cm)
    i: int                           # X-axis discrete index at this level
    j: int                           # Y-axis discrete index at this level
    resolution: float                # Cell side length in meters
    x_min: float                     # Spatial bounding box min X
    y_min: float                     # Spatial bounding box min Y
    x_max: float                     # Spatial bounding box max X
    y_max: float                     # Spatial bounding box max Y
    center_x: float                  # Center X coordinate
    center_y: float                  # Center Y coordinate
    
    # 2.5D Multi-Layer Attributes
    elevation: float = 0.0           # Robust median surface height
    elevation_var: float = 0.0       # Elevation variance
    elevation_min: float = 0.0
    elevation_max: float = 0.0
    point_count: int = 0
    
    # Semantic & Occupancy
    semantic_class: int = 0          # Dominant semantic class ID
    semantic_name: str = "Unknown"
    confidence: float = 0.5          # Classification confidence [0..1]
    class_probs: Dict[int, float] = field(default_factory=lambda: {c: 0.0 for c in range(8)})
    occupancy_prob: float = 0.5      # Occupancy probability [0..1]
    is_dynamic: bool = False
    dynamic_prob: float = 0.0
    object_id: Optional[int] = None
    velocity: Tuple[float, float] = (0.0, 0.0)
    
    # Perception-Guided Metrics
    geometry_complexity: float = 0.0
    complexity_category: str = "LOW"
    importance_score: float = 0.0
    danger_score: float = 0.0
    danger_probability: float = 0.0
    danger_level: str = "SAFE"
    timestamp: float = field(default_factory=time.time)

    @property
    def key(self) -> Tuple[int, int, int]:
        return (self.level, self.i, self.j)


class AdaptiveGrid:
    """Hierarchical Adaptive 2.5D Grid Map.
    Zero-gap quadtree tiling with multi-layer elevation and semantic fusion.
    """

    def __init__(
        self,
        x_min: float = -40.0,
        x_max: float = 60.0,
        y_min: float = -40.0,
        y_max: float = 40.0,
        base_resolution: float = 0.05,  # Level 0 = 0.05 m
    ):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.base_res = base_resolution

        # Level resolutions: Level L = base_res * (2^L)
        self.resolutions = {lvl: base_resolution * (2**lvl) for lvl in range(5)}
        self.policy = ResolutionPolicy()
        
        # Storage for active leaf cells: key (level, i, j) -> GridCell
        self.cells: Dict[Tuple[int, int, int], GridCell] = {}
        # Set of parent keys that have active children (internal nodes)
        self.subdivided_parents: Set[Tuple[int, int, int]] = set()

    def point_to_resolution(self, x: float, y: float, level: int) -> float:
        """Returns the resolution in meters for a given level."""
        return self.resolutions.get(level, self.resolutions[4])

    def point_to_indices(self, x: float, y: float, level: int) -> Tuple[int, int]:
        """Maps continuous (x, y) coordinates to integer grid indices at level."""
        res = self.resolutions[level]
        i = int(np.floor((x - self.x_min) / res))
        j = int(np.floor((y - self.y_min) / res))
        return i, j

    def indices_to_bounds(self, i: int, j: int, level: int) -> Tuple[float, float, float, float]:
        """Calculates spatial bounding box (x_min, y_min, x_max, y_max) of cell."""
        res = self.resolutions[level]
        x0 = self.x_min + i * res
        y0 = self.y_min + j * res
        return x0, y0, x0 + res, y0 + res

    def point_to_cell(self, x: float, y: float, level: int) -> GridCell:
        """Creates or retrieves a GridCell for continuous (x, y) at level."""
        i, j = self.point_to_indices(x, y, level)
        key = (level, i, j)
        if key in self.cells:
            return self.cells[key]
        
        res = self.resolutions[level]
        x0, y0, x1, y1 = self.indices_to_bounds(i, j, level)
        cell = GridCell(
            level=level,
            i=i,
            j=j,
            resolution=res,
            x_min=x0,
            y_min=y0,
            x_max=x1,
            y_max=y1,
            center_x=x0 + res / 2.0,
            center_y=y0 + res / 2.0,
        )
        return cell

    def cell_parent(self, cell: GridCell) -> Optional[Tuple[int, int, int]]:
        """Returns the parent cell key at level + 1."""
        if cell.level >= 4:
            return None
        p_lvl = cell.level + 1
        p_i = cell.i // 2
        p_j = cell.j // 2
        return (p_lvl, p_i, p_j)

    def cell_children(self, cell: GridCell) -> List[Tuple[int, int, int]]:
        """Returns the 4 quadtree children cell keys at level - 1."""
        if cell.level <= 0:
            return []
        c_lvl = cell.level - 1
        c_i = cell.i * 2
        c_j = cell.j * 2
        return [
            (c_lvl, c_i, c_j),
            (c_lvl, c_i + 1, c_j),
            (c_lvl, c_i, c_j + 1),
            (c_lvl, c_i + 1, c_j + 1),
        ]

    def neighbor_lookup(self, cell: GridCell) -> List[Tuple[int, int, int]]:
        """Returns 8-connected neighbor cell keys at the same level."""
        nbrs = []
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                nbrs.append((cell.level, cell.i + di, cell.j + dj))
        return nbrs

    def refine_cell(self, cell: GridCell) -> List[GridCell]:
        """Subdivides a cell into 4 finer children (level -> level - 1).
        Seamlessly removes parent from active leaf cells and instantiates 4 children.
        """
        if cell.level <= 0:
            return [cell]

        parent_key = cell.key
        children_keys = self.cell_children(cell)
        children_cells = []

        for c_lvl, c_i, c_j in children_keys:
            res = self.resolutions[c_lvl]
            x0, y0, x1, y1 = self.indices_to_bounds(c_i, c_j, c_lvl)
            child = GridCell(
                level=c_lvl,
                i=c_i,
                j=c_j,
                resolution=res,
                x_min=x0,
                y_min=y0,
                x_max=x1,
                y_max=y1,
                center_x=x0 + res / 2.0,
                center_y=y0 + res / 2.0,
                elevation=cell.elevation,
                elevation_var=cell.elevation_var,
                semantic_class=cell.semantic_class,
                semantic_name=cell.semantic_name,
                confidence=cell.confidence,
                class_probs=dict(cell.class_probs),
                occupancy_prob=cell.occupancy_prob,
                is_dynamic=cell.is_dynamic,
                dynamic_prob=cell.dynamic_prob,
                object_id=cell.object_id,
                velocity=cell.velocity,
                geometry_complexity=cell.geometry_complexity,
                complexity_category=cell.complexity_category,
                importance_score=cell.importance_score,
                danger_score=cell.danger_score,
                danger_probability=cell.danger_probability,
                danger_level=cell.danger_level,
            )
            self.cells[(c_lvl, c_i, c_j)] = child
            children_cells.append(child)

        # Remove parent from active leaves and mark as subdivided
        if parent_key in self.cells:
            del self.cells[parent_key]
        self.subdivided_parents.add(parent_key)

        return children_cells

    def merge_cells(self, parent_key: Tuple[int, int, int]) -> Optional[GridCell]:
        """Merges 4 sibling children back into their coarser parent cell."""
        p_lvl, p_i, p_j = parent_key
        if p_lvl > 4 or p_lvl <= 0:
            return None

        c_lvl = p_lvl - 1
        c_keys = [
            (c_lvl, p_i * 2, p_j * 2),
            (c_lvl, p_i * 2 + 1, p_j * 2),
            (c_lvl, p_i * 2, p_j * 2 + 1),
            (c_lvl, p_i * 2 + 1, p_j * 2 + 1),
        ]

        # Gather active children
        present_children = [self.cells[k] for k in c_keys if k in self.cells]
        if not present_children:
            return None

        # Instantiate parent
        res = self.resolutions[p_lvl]
        x0, y0, x1, y1 = self.indices_to_bounds(p_i, p_j, p_lvl)
        
        # Fuse attributes across children
        avg_elev = float(np.mean([c.elevation for c in present_children]))
        avg_var = float(np.mean([c.elevation_var for c in present_children]))
        avg_imp = float(np.mean([c.importance_score for c in present_children]))
        avg_danger = float(np.mean([c.danger_score for c in present_children]))
        dom_sem = present_children[0].semantic_class

        parent_cell = GridCell(
            level=p_lvl,
            i=p_i,
            j=p_j,
            resolution=res,
            x_min=x0,
            y_min=y0,
            x_max=x1,
            y_max=y1,
            center_x=x0 + res / 2.0,
            center_y=y0 + res / 2.0,
            elevation=avg_elev,
            elevation_var=avg_var,
            semantic_class=dom_sem,
            semantic_name=CLASS_NAMES.get(dom_sem, "Unknown"),
            importance_score=avg_imp,
            danger_score=avg_danger,
            danger_level=present_children[0].danger_level,
        )

        # Remove children from active leaves
        for k in c_keys:
            if k in self.cells:
                del self.cells[k]

        self.cells[parent_key] = parent_cell
        self.subdivided_parents.discard(parent_key)
        return parent_cell

    def update_elevation(self, cell: GridCell, points: np.ndarray):
        """Updates elevation statistics from LiDAR point cloud array."""
        if len(points) == 0:
            return
        z_vals = points[:, 2]
        stats = ElevationLayer.compute_elevation_stats(z_vals)
        cell.elevation = stats["elevation"]
        cell.elevation_var = stats["variance"]
        cell.elevation_min = stats["min"]
        cell.elevation_max = stats["max"]
        cell.point_count = len(points)

    def update_semantics(self, cell: GridCell, labels: np.ndarray, confidences: np.ndarray):
        """Updates semantic class and probability distribution."""
        if len(labels) == 0:
            return
        sem_res = SemanticLayer.aggregate_points_semantics(labels, confidences)
        cell.semantic_class = sem_res["dominant_class"]
        cell.semantic_name = sem_res["dominant_name"]
        cell.confidence = sem_res["confidence"]
        cell.class_probs = sem_res["class_probs"]

    def update_occupancy(self, cell: GridCell, num_points: int, is_dynamic: bool = False):
        """Updates occupancy probability and dynamic status."""
        # Simple log-odds occupancy update
        if num_points > 0:
            cell.occupancy_prob = min(cell.occupancy_prob + 0.35, 0.98)
        else:
            cell.occupancy_prob = max(cell.occupancy_prob - 0.10, 0.02)
        cell.is_dynamic = is_dynamic
        cell.dynamic_prob = 0.90 if is_dynamic else 0.05

    def get_active_cells(self) -> List[GridCell]:
        """Returns all currently active leaf grid cells."""
        return list(self.cells.values())

    def get_cell_count_by_level(self) -> Dict[int, int]:
        """Returns tally of active cells at each resolution level."""
        counts = {lvl: 0 for lvl in range(5)}
        for cell in self.cells.values():
            counts[cell.level] += 1
        return counts

    def compute_equivalent_uniform_cell_count(self) -> int:
        """Computes the theoretical number of cells required if a uniform 5 cm grid were used."""
        width = self.x_max - self.x_min
        height = self.y_max - self.y_min
        uniform_cols = int(width / self.base_res)
        uniform_rows = int(height / self.base_res)
        return uniform_cols * uniform_rows
