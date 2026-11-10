"""
costmap.py — Entrypoint and public interface for progressive point cloud accumulation
and gradient signed distance field (SDF) costmap generation.

Pipeline:
  ZED Camera (XYZRGBA + Pose)
      │
      ▼
  Frame Filtering & Coordinate Transform (world frame)
      │
      ▼
  Incremental Voxel Accumulation  ←─── z-floor filtering (drop ground)
      │
      ▼
  Progressive Denoising
    ├── Statistical Outlier Removal (SOR) on new chunks
    └── Periodic global voxel downsampling
      │
      ▼
  2D Occupancy Grid projection  (max-z or density threshold)
      │
      ▼
  Euclidean Distance Transform  → raw SDF
      │
      ▼
  Sobel Gradient  →  ∇SDF (costmap)
      │
      ▼
  A* / path planner reads gradient costmap

Visualizers (run in background threads):
  • PointCloudVisualizer  — PyVista 3-D live point cloud + robot position marker
  • CostmapVisualizer     — Matplotlib 2-D panel:
                              [Occupancy | SDF | Gradient magnitude | Cost]
                            Each panel shows the robot as a coloured arrow.

Usage:
    python costmap.py                    # live ZED camera, all visualizers on
    python costmap.py --svo FILE         # replay from .svo2 file
    python costmap.py --no-vis           # headless / no windows
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Ensure autonomy directory is on sys.path for direct script execution
_MODULE_DIR = str(Path(__file__).resolve().parent)
if _MODULE_DIR not in sys.path:
    sys.path.insert(0, _MODULE_DIR)

try:
    from .config import CostmapConfig, load_config
    from .accumulator import PointCloudAccumulator
    from .sdf_costmap import GradientSDFCostmap
    from .visualizers import PointCloudVisualizer, CostmapVisualizer
    from .node import AutonomyNode
except ImportError:
    from config import CostmapConfig, load_config
    from accumulator import PointCloudAccumulator
    from sdf_costmap import GradientSDFCostmap
    from visualizers import PointCloudVisualizer, CostmapVisualizer
    from node import AutonomyNode

__all__ = [
    "CostmapConfig",
    "load_config",
    "PointCloudAccumulator",
    "GradientSDFCostmap",
    "PointCloudVisualizer",
    "CostmapVisualizer",
    "AutonomyNode",
    "main",
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Lunabotics Gradient SDF Costmap Node")
    parser.add_argument("--svo",       type=str,   default=None,  help="Path to .svo2 file (replay mode)")
    parser.add_argument("--voxel",     type=float, default=0.05,  help="Voxel size in metres (default: 0.05)")
    parser.add_argument("--max-depth", type=float, default=8.0,   help="Maximum ZED depth in metres")
    parser.add_argument("--map-res",   type=float, default=0.05,  help="Costmap resolution in metres per cell")
    parser.add_argument("--no-vis",    action="store_true",        help="Disable all visualizers (headless mode)")
    args = parser.parse_args()

    cfg = CostmapConfig(
        voxel_size_m     = args.voxel,
        max_depth_m      = args.max_depth,
        map_resolution_m = args.map_res,
    )

    node = AutonomyNode(cfg, svo_path=args.svo, visualize=not args.no_vis)
    node.run()


if __name__ == "__main__":
    main()
