"""Base Perception Model Adapter Interface.
Provides unified abstraction for 3D semantic segmentation and detection backends.
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np

from app.core.perception import PerceptionResult


class BasePerceptionAdapter(ABC):
    """Abstract base class for modular perception models."""

    def __init__(self, name: str, mode: str = "DEMO/SIMULATED"):
        self.name = name
        self.mode = mode

    @abstractmethod
    def is_available(self) -> bool:
        """Returns True if model weights and execution environment are ready."""
        pass

    @abstractmethod
    def predict(
        self,
        points: np.ndarray,
        intensities: Optional[np.ndarray] = None,
        ground_truth_labels: Optional[np.ndarray] = None,
    ) -> PerceptionResult:
        """Runs inference or deterministic demo simulation."""
        pass
