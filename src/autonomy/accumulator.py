"""
accumulator.py — Point cloud accumulation with progressive denoising.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d

try:
    from .config import CostmapConfig
except ImportError:
    from config import CostmapConfig


class PointCloudAccumulator:
    """
    Maintains a globally-consistent, progressively-denoised point cloud.

    Each captured frame is:
      1. Filtered (NaN/Inf removal)
      2. Subsampled & RGB unpacked
      3. Transformed into the world frame via camera pose
      4. Filtered by height (removes ground floor & ceiling returns)
      5. Merged into the global map via voxel downsampling
      6. Periodically denoised with Statistical Outlier Removal (SOR)
    """

    def __init__(self, cfg: CostmapConfig) -> None:
        self.cfg = cfg
        self.global_pcd = o3d.geometry.PointCloud()
        self._frame_count = 0

    # ------------------------------------------------------------------ #
    #  Public API                                                        #
    # ------------------------------------------------------------------ #

    def ingest_frame(
        self,
        pc_np: np.ndarray,          # (H, W, 4) XYZRGBA from ZED
        pose_matrix: np.ndarray,    # (4, 4) camera -> world transform
    ) -> None:
        """Merge one ZED point-cloud frame into the global map."""
        frame_pcd = self._extract_pcd(pc_np)
        if len(frame_pcd.points) == 0:
            return

        # Transform to world frame
        frame_pcd.transform(pose_matrix)

        # Height filter (removes ground plane + ceiling returns)
        pts = np.asarray(frame_pcd.points)
        mask = (pts[:, 2] >= self.cfg.floor_z_min_m) & (pts[:, 2] <= self.cfg.ceil_z_max_m)
        frame_pcd = frame_pcd.select_by_index(np.where(mask)[0])

        if len(frame_pcd.points) == 0:
            return

        # Accumulate with voxel downsampling
        self.global_pcd += frame_pcd
        self.global_pcd = self.global_pcd.voxel_down_sample(self.cfg.voxel_size_m)

        self._frame_count += 1

        # Periodic denoising
        if self._frame_count % self.cfg.sor_every_n_frames == 0:
            self._denoise()

    @property
    def points(self) -> np.ndarray:
        """Return (N, 3) numpy array of current accumulated points."""
        return np.asarray(self.global_pcd.points)

    @property
    def frame_count(self) -> int:
        return self._frame_count

    # ------------------------------------------------------------------ #
    #  Internal Helpers                                                  #
    # ------------------------------------------------------------------ #

    def _extract_pcd(self, pc_np: np.ndarray) -> o3d.geometry.PointCloud:
        """Convert raw ZED XYZRGBA array to an Open3D PointCloud."""
        cfg = self.cfg
        points = pc_np[:, :, :3].reshape(-1, 3)
        colors_raw = pc_np[:, :, 3].reshape(-1)

        # Validity mask
        valid = ~(np.isnan(points).any(axis=1) | np.isinf(points).any(axis=1))
        points = points[valid]
        colors_raw = colors_raw[valid]

        # Subsample
        points = points[::cfg.frame_subsample]
        colors_raw = colors_raw[::cfg.frame_subsample]

        if len(points) == 0:
            return o3d.geometry.PointCloud()

        # Decode packed RGBA -> float RGB
        rgba = colors_raw.view(np.uint32)
        r = (rgba & 0x000000FF).astype(np.float64) / 255.0
        g = ((rgba & 0x0000FF00) >> 8).astype(np.float64) / 255.0
        b = ((rgba & 0x00FF0000) >> 16).astype(np.float64) / 255.0
        rgb = np.stack([r, g, b], axis=-1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(rgb)
        return pcd

    def _denoise(self) -> None:
        """
        In-place Statistical Outlier Removal.
        Points whose mean distance to k-neighbours exceeds
        (global_mean + std_ratio * global_std) are removed.
        """
        if len(self.global_pcd.points) < self.cfg.sor_nb_neighbors + 1:
            return
        pcd_clean, _ = self.global_pcd.remove_statistical_outlier(
            nb_neighbors=self.cfg.sor_nb_neighbors,
            std_ratio=self.cfg.sor_std_ratio,
        )
        self.global_pcd = pcd_clean
