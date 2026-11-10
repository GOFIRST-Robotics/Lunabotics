"""
node.py — Top-level AutonomyNode orchestrating ZED sensor, SLAM, and costmap updates.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import pyzed.sl as sl

try:
    from .config import CostmapConfig
    from .accumulator import PointCloudAccumulator
    from .sdf_costmap import GradientSDFCostmap
    from .visualizers import PointCloudVisualizer, CostmapVisualizer
except ImportError:
    from config import CostmapConfig
    from accumulator import PointCloudAccumulator
    from sdf_costmap import GradientSDFCostmap
    from visualizers import PointCloudVisualizer, CostmapVisualizer


class AutonomyNode:
    """
    Top-level orchestrator. Connects ZED, runs the accumulation/denoising
    loop, and drives costmap updates.

    Optional background visualizers:
      - PointCloudVisualizer: PyVista 3D cloud + robot marker
      - CostmapVisualizer: Matplotlib 2D 4-panel costmap
    """

    def __init__(
        self,
        cfg: CostmapConfig,
        svo_path: Optional[str] = None,
        visualize: bool = True,
    ) -> None:
        self.cfg = cfg
        self.accum = PointCloudAccumulator(cfg)
        self.costmap = GradientSDFCostmap(cfg)

        # Background visualizers (None when headless)
        self._pv_vis: Optional[PointCloudVisualizer] = None
        self._mp_vis: Optional[CostmapVisualizer] = None
        if visualize:
            self._pv_vis = PointCloudVisualizer(cfg)
            self._mp_vis = CostmapVisualizer(cfg)

        # ZED Camera setup
        init_params = sl.InitParameters()
        init_params.camera_resolution = cfg.resolution
        init_params.depth_mode = cfg.depth_mode
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_maximum_distance = cfg.max_depth_m

        if svo_path:
            init_params.set_from_svo_file(svo_path)

        self.zed = sl.Camera()
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {err}")

        # Positional tracking
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_area_memory = True
        err = self.zed.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.zed.close()
            raise RuntimeError(f"Positional tracking failed: {err}")

        self.runtime = sl.RuntimeParameters()
        self.runtime.confidence_threshold = cfg.confidence_threshold

        self._pc_mat = sl.Mat()
        self._pose = sl.Pose()
        self._running = False

    def run(self) -> None:
        """Main sensor loop. Ctrl-C to stop."""
        self._running = True
        vis_flag = self._pv_vis is not None
        print(f"Autonomy node running [visualizers={'on' if vis_flag else 'off'}]. Press Ctrl-C to stop.")
        try:
            while self._running:
                self._step()
        except KeyboardInterrupt:
            print("\nShutting down.")
        finally:
            self._cleanup()

    def get_costmap(self) -> np.ndarray:
        """Return the latest gradient-SDF costmap for the path planner."""
        return self.costmap.cost

    # ------------------------------------------------------------------ #
    #  Internal Loop                                                     #
    # ------------------------------------------------------------------ #

    def _step(self) -> None:
        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        # Retrieve raw point cloud
        self.zed.retrieve_measure(self._pc_mat, sl.MEASURE.XYZRGBA)
        pc_np = self._pc_mat.get_data()

        # Retrieve camera pose
        tracking_state = self.zed.get_position(self._pose, sl.REFERENCE_FRAME.WORLD)
        if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            return

        pose_matrix = self._pose.pose_data().m

        # Extract robot world position and heading (yaw)
        robot_pos = pose_matrix[:3, 3]
        robot_heading = float(np.arctan2(pose_matrix[1, 0], pose_matrix[0, 0]))

        # Accumulate + denoise
        self.accum.ingest_frame(pc_np, pose_matrix)

        # Update costmap + push to visualizers periodically
        if self.accum.frame_count % self.cfg.vis_update_every_n_frames == 0:
            pts = self.accum.points
            if len(pts) > 0:
                self.costmap.update(pts)
                self._log_status()

                if self._pv_vis is not None:
                    self._pv_vis.push(self.accum.global_pcd, robot_pos, robot_heading)

                if self._mp_vis is not None:
                    self._mp_vis.push(self.costmap, robot_pos, robot_heading)

    def _log_status(self) -> None:
        n_pts = len(self.accum.points)
        occ = self.costmap.occupancy.sum()
        sdf_m = self.costmap.sdf.mean()
        print(
            f"[frame {self.accum.frame_count:05d}]  "
            f"cloud={n_pts:>7,} pts  "
            f"occupied={occ:>5} cells  "
            f"mean_sdf={sdf_m:.3f} m"
        )

    def _cleanup(self) -> None:
        if self._pv_vis is not None:
            try:
                self._pv_vis.close()
            except Exception:
                pass
        if self._mp_vis is not None:
            try:
                self._mp_vis.close()
            except Exception:
                pass
        try:
            self._pc_mat.free()
        except Exception:
            pass
        try:
            self.zed.disable_positional_tracking()
        except Exception:
            pass
        try:
            self.zed.close()
        except Exception:
            pass
