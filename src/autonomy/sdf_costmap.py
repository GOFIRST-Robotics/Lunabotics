"""
sdf_costmap.py — 2D Occupancy Grid and Gradient Signed Distance Field (SDF) costmap.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.ndimage import distance_transform_edt, sobel

try:
    from .config import CostmapConfig
except ImportError:
    from config import CostmapConfig


class GradientSDFCostmap:
    """
    Projects a 3D point cloud onto a 2D occupancy grid, then computes:

        SDF(cell) = distance to nearest occupied cell (metres)
        grad(SDF) = spatial gradient of the SDF

    The composite cost for path planning (e.g. A*) is:
        cost(cell) = exp(-alpha * SDF) * |grad(SDF)|
    """

    def __init__(self, cfg: CostmapConfig) -> None:
        self.cfg = cfg

        # Grid dimensions
        self.cols = int(cfg.map_width_m / cfg.map_resolution_m)
        self.rows = int(cfg.map_height_m / cfg.map_resolution_m)

        # Persistent grids
        self._occupancy: np.ndarray = np.zeros((self.rows, self.cols), dtype=np.int32)
        self._sdf: Optional[np.ndarray] = None       # (rows, cols) in metres
        self._grad_x: Optional[np.ndarray] = None
        self._grad_y: Optional[np.ndarray] = None
        self._cost: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ #
    #  Public API                                                        #
    # ------------------------------------------------------------------ #

    def update(self, points_world: np.ndarray) -> None:
        """
        Rebuild occupancy from the accumulated world points, then compute
        SDF and gradient costmap.
        """
        self._build_occupancy(points_world)
        self._compute_sdf()
        self._compute_gradient()

    @property
    def occupancy(self) -> np.ndarray:
        """Binary grid: 1 = obstacle, 0 = free."""
        return (self._occupancy >= self.cfg.occupancy_density).astype(np.uint8)

    @property
    def sdf(self) -> np.ndarray:
        """Signed Distance Field in metres (distance to nearest obstacle)."""
        return self._sdf if self._sdf is not None else np.zeros((self.rows, self.cols))

    @property
    def gradient_x(self) -> np.ndarray:
        return self._grad_x if self._grad_x is not None else np.zeros((self.rows, self.cols))

    @property
    def gradient_y(self) -> np.ndarray:
        return self._grad_y if self._grad_y is not None else np.zeros((self.rows, self.cols))

    @property
    def cost(self) -> np.ndarray:
        """Composite costmap in range [0, 1] for path planning."""
        return self._cost if self._cost is not None else np.zeros((self.rows, self.cols))

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        """Convert world coordinates (x, y) in metres -> (row, col) grid indices."""
        cfg = self.cfg
        col = int((x_m - cfg.map_origin_x_m) / cfg.map_resolution_m)
        row = int((y_m - cfg.map_origin_y_m) / cfg.map_resolution_m)
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert (row, col) grid indices -> world coordinates (x, y) in metres."""
        cfg = self.cfg
        x = col * cfg.map_resolution_m + cfg.map_origin_x_m
        y = row * cfg.map_resolution_m + cfg.map_origin_y_m
        return x, y

    # ------------------------------------------------------------------ #
    #  Internal Helpers                                                  #
    # ------------------------------------------------------------------ #

    def _build_occupancy(self, points: np.ndarray) -> None:
        """
        Project 3D world points onto the 2D XY grid using vectorised binning.
        """
        cfg = self.cfg
        grid = np.zeros((self.rows, self.cols), dtype=np.int32)

        # Project X, Y -> col, row
        cols = ((points[:, 0] - cfg.map_origin_x_m) / cfg.map_resolution_m).astype(int)
        rows = ((points[:, 1] - cfg.map_origin_y_m) / cfg.map_resolution_m).astype(int)

        # Bounds check
        in_bounds = (
            (rows >= 0) & (rows < self.rows) &
            (cols >= 0) & (cols < self.cols)
        )
        rows, cols = rows[in_bounds], cols[in_bounds]

        np.add.at(grid, (rows, cols), 1)
        self._occupancy = grid

    def _compute_sdf(self) -> None:
        """
        Compute Euclidean Distance Transform on free-space mask and scale to metres.
        """
        free_mask = (self._occupancy < self.cfg.occupancy_density)
        dist_px = distance_transform_edt(free_mask)
        dist_m = dist_px * self.cfg.map_resolution_m
        self._sdf = np.minimum(dist_m, self.cfg.sdf_truncation_m)

    def _compute_gradient(self) -> None:
        """
        Compute Sobel spatial gradient of the SDF and blend into a composite cost.
        """
        sdf = self._sdf
        gx = sobel(sdf, axis=1)   # dSDF / dx (cols)
        gy = sobel(sdf, axis=0)   # dSDF / dy (rows)

        self._grad_x = gx * self.cfg.gradient_scale
        self._grad_y = gy * self.cfg.gradient_scale

        grad_mag = np.sqrt(gx**2 + gy**2) * self.cfg.gradient_scale

        alpha = 2.0 / max(self.cfg.sdf_truncation_m, 1e-6)
        proximity = np.exp(-alpha * sdf)

        # Normalise and blend into [0, 1]
        self._cost = np.clip(proximity + grad_mag / (grad_mag.max() + 1e-9), 0.0, 1.0)
