"""
config.py — Configuration parameters for Lunabotics Autonomy Costmap & SLAM.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import pyzed.sl as sl

CONFIG_PATH = Path(__file__).parent / "config.json"


def load_config() -> dict:
    """Load JSON config if present, returning a dictionary."""
    if CONFIG_PATH.exists():
        with open(CONFIG_PATH) as f:
            return json.load(f)
    return {}


@dataclass
class CostmapConfig:
    # --- ZED Camera ---
    resolution: sl.RESOLUTION = sl.RESOLUTION.HD720
    depth_mode: sl.DEPTH_MODE = sl.DEPTH_MODE.NEURAL
    max_depth_m: float = 8.0
    confidence_threshold: int = 50

    # --- Point Cloud Accumulation ---
    voxel_size_m: float = 0.05          # Voxel downsampling resolution in metres
    frame_subsample: int = 4            # Subsample factor per frame
    floor_z_min_m: float = -0.10       # Drop points below this (ground noise)
    ceil_z_max_m: float = 2.00         # Drop points above this (sky/ceiling)

    # --- Statistical Outlier Removal (SOR) ---
    sor_nb_neighbors: int = 20          # k-nearest neighbours
    sor_std_ratio: float = 2.0          # Outlier threshold (mean + ratio * std)
    sor_every_n_frames: int = 10        # Run SOR every N frames

    # --- Costmap / 2D Occupancy Grid ---
    map_resolution_m: float = 0.05     # Metres per cell
    map_width_m: float = 20.0          # Total map width (X)
    map_height_m: float = 20.0         # Total map height (Y)
    map_origin_x_m: float = -10.0      # World X of cell (0, 0)
    map_origin_y_m: float = -10.0      # World Y of cell (0, 0)
    occupancy_density: int = 2         # Minimum points per cell to mark occupied
    sdf_truncation_m: float = 2.0      # Clamp SDF beyond this distance

    # --- Gradient SDF ---
    gradient_scale: float = 1.0        # Multiplier for gradient magnitude

    # --- Visualisation ---
    vis_update_every_n_frames: int = 5     # Frequency of costmap redraws
    vis_pv_update_every_n_frames: int = 5  # Frequency of 3D point cloud redraws
    vis_robot_size_m: float = 0.5          # Robot footprint radius for visual markers
