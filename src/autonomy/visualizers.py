"""
visualizers.py — 3D Point Cloud and 2D Gradient SDF Costmap live visualizers.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional

import matplotlib
# Select interactive backend if display available, otherwise headless Agg
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
else:
    for backend in ("TkAgg", "QtAgg"):
        try:
            matplotlib.use(backend)
            break
        except Exception:
            pass

import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pyvista as pv

try:
    from .config import CostmapConfig
    from .sdf_costmap import GradientSDFCostmap
except ImportError:
    from config import CostmapConfig
    from sdf_costmap import GradientSDFCostmap


class PointCloudVisualizer:
    """
    PyVista 3D live point cloud viewer running in a background thread.

    Displays:
      - Accumulated coloured point cloud (world frame)
      - Robot position as a cyan sphere
      - Robot heading as a yellow directional arrow
      - Live frame/point count stats in window title
    """

    def __init__(self, cfg: CostmapConfig) -> None:
        self.cfg = cfg
        self._lock = threading.Lock()
        self._pts: Optional[np.ndarray] = None
        self._cols: Optional[np.ndarray] = None
        self._robot_pos: Optional[np.ndarray] = None
        self._robot_heading: float = 0.0
        self._dirty = False
        self._alive = True

        self._thread = threading.Thread(target=self._render_loop, daemon=True)
        self._thread.start()

    def push(
        self,
        pcd: o3d.geometry.PointCloud,
        robot_pos: np.ndarray,
        robot_heading: float,
    ) -> None:
        """Hand off latest data to the render thread. Non-blocking."""
        if not self._alive:
            return
        pts = np.asarray(pcd.points)
        cols = np.asarray(pcd.colors) if pcd.has_colors() else np.ones((len(pts), 3))
        with self._lock:
            self._pts = pts.copy()
            self._cols = cols.copy()
            self._robot_pos = robot_pos.copy()
            self._robot_heading = robot_heading
            self._dirty = True

    def close(self) -> None:
        self._alive = False

    def _render_loop(self) -> None:
        try:
            pl = pv.Plotter(title="Lunabotics — Point Cloud (3-D)")
            pl.set_background("black")
            pl.show_axes()
            pl.show(auto_close=False, interactive_update=True)
        except Exception as e:
            print(f"[PointCloudVisualizer] Failed to initialize 3D display: {e}")
            self._alive = False
            return

        cloud_actor = None
        robot_actor = None
        arrow_actor = None

        try:
            while self._alive and not pl.render_window.GetInteractor().GetDone():
                with self._lock:
                    dirty = self._dirty
                    if dirty:
                        pts = self._pts
                        cols = self._cols
                        robot_pos = self._robot_pos
                        robot_heading = self._robot_heading
                        self._dirty = False

                if dirty and pts is not None and len(pts) > 0:
                    # Update point cloud mesh
                    cloud = pv.PolyData(pts)
                    rgb255 = (np.clip(cols, 0, 1) * 255).astype(np.uint8)
                    cloud.point_data["RGB"] = rgb255

                    if cloud_actor is not None:
                        pl.remove_actor(cloud_actor, render=False)
                    cloud_actor = pl.add_mesh(
                        cloud,
                        scalars="RGB",
                        rgb=True,
                        point_size=3,
                        render_points_as_spheres=False,
                    )

                    # Update robot sphere & heading arrow
                    if robot_pos is not None:
                        r = self.cfg.vis_robot_size_m
                        sphere = pv.Sphere(radius=r, center=robot_pos.tolist())
                        if robot_actor is not None:
                            pl.remove_actor(robot_actor, render=False)
                        robot_actor = pl.add_mesh(sphere, color="cyan", opacity=0.85)

                        arrow_len = r * 3.0
                        direction = np.array([
                            np.cos(robot_heading),
                            np.sin(robot_heading),
                            0.0,
                        ])
                        arrow = pv.Arrow(
                            start=robot_pos.tolist(),
                            direction=direction.tolist(),
                            scale=arrow_len,
                        )
                        if arrow_actor is not None:
                            pl.remove_actor(arrow_actor, render=False)
                        arrow_actor = pl.add_mesh(arrow, color="yellow")

                    n_pts = len(pts)
                    pl.render_window.SetWindowName(
                        f"Lunabotics — Point Cloud  [{n_pts:,} pts]"
                    )

                pl.update()
                time.sleep(0.03)
        except Exception as e:
            print(f"[PointCloudVisualizer] 3D display event error: {e}")
        finally:
            try:
                pl.close()
            except Exception:
                pass
            self._alive = False


class CostmapVisualizer:
    """
    Matplotlib 2D costmap panel running in a daemon thread:

        [ Occupancy  |  SDF (m)      ]
        [ Grad Mag   |  Costmap (A*) ]

    Overlays robot circle and heading arrow on all panels, plus a gradient
    vector quiver on the Costmap panel.
    """

    _CMAPS = {
        "occupancy": "binary",
        "sdf": "plasma",
        "grad": "inferno",
        "cost": "RdYlGn_r",
    }

    def __init__(self, cfg: CostmapConfig) -> None:
        self.cfg = cfg
        self._lock = threading.Lock()
        self._data: Optional[dict] = None
        self._dirty = False
        self._alive = True

        self._thread = threading.Thread(target=self._render_loop, daemon=True)
        self._thread.start()

    def push(
        self,
        costmap: GradientSDFCostmap,
        robot_pos_world: np.ndarray,
        robot_heading: float,
    ) -> None:
        """Snapshot costmap arrays and robot state for the render thread."""
        if not self._alive:
            return
        rx, ry = robot_pos_world[0], robot_pos_world[1]
        row, col = costmap.world_to_cell(rx, ry)
        grad_mag = np.sqrt(costmap.gradient_x**2 + costmap.gradient_y**2)

        with self._lock:
            self._data = {
                "occupancy": costmap.occupancy.copy(),
                "sdf": costmap.sdf.copy(),
                "grad_mag": grad_mag.copy(),
                "cost": costmap.cost.copy(),
                "grad_x": costmap.gradient_x.copy(),
                "grad_y": costmap.gradient_y.copy(),
                "robot_row": row,
                "robot_col": col,
                "robot_heading": robot_heading,
                "map_origin_x": costmap.cfg.map_origin_x_m,
                "map_origin_y": costmap.cfg.map_origin_y_m,
                "map_res": costmap.cfg.map_resolution_m,
                "sdf_max": costmap.cfg.sdf_truncation_m,
            }
            self._dirty = True

    def close(self) -> None:
        self._alive = False

    def _render_loop(self) -> None:
        is_headless = False
        try:
            plt.ion()
            fig, axes = plt.subplots(2, 2, figsize=(13, 10), facecolor="#1a1a2e")
            if fig.canvas.manager is not None:
                fig.canvas.manager.set_window_title("Lunabotics — Costmap (2-D)")
            fig.suptitle("Gradient SDF Costmap", color="white", fontsize=14, fontweight="bold")

            titles = ["Occupancy", "SDF (m)", "∇SDF Magnitude", "Cost (A*)"]
            cmaps = [self._CMAPS[k] for k in ("occupancy", "sdf", "grad", "cost")]

            ax_list: list[plt.Axes] = [axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]]
            imgs: list[plt.AxesImage] = []
            quiver_plots: list = [None] * 4
            robot_patches: list = []

            for ax, title, cmap in zip(ax_list, titles, cmaps):
                ax.set_facecolor("#0d0d1a")
                ax.set_title(title, color="white", fontsize=11)
                ax.tick_params(colors="gray", labelsize=7)
                for spine in ax.spines.values():
                    spine.set_edgecolor("#444")
                img = ax.imshow(
                    np.zeros((10, 10)),
                    cmap=cmap,
                    origin="lower",
                    interpolation="nearest",
                    aspect="auto",
                )
                imgs.append(img)
                cb = fig.colorbar(img, ax=ax, fraction=0.046, pad=0.04)
                cb.ax.tick_params(colors="gray", labelsize=7)
                cb.outline.set_edgecolor("#444")

            fig.tight_layout(rect=(0, 0, 1, 0.95))
            if matplotlib.get_backend().lower() == "agg":
                is_headless = True
            else:
                plt.pause(0.01)
        except Exception as e:
            # Fallback to headless Agg backend to still output costmap_latest.png
            print(f"[CostmapVisualizer] GUI window unavailable ({e}). Falling back to headless saving: costmap_latest.png")
            try:
                matplotlib.use("Agg", force=True)
                fig, axes = plt.subplots(2, 2, figsize=(13, 10), facecolor="#1a1a2e")
                fig.suptitle("Gradient SDF Costmap", color="white", fontsize=14, fontweight="bold")
                titles = ["Occupancy", "SDF (m)", "∇SDF Magnitude", "Cost (A*)"]
                cmaps = [self._CMAPS[k] for k in ("occupancy", "sdf", "grad", "cost")]
                ax_list = [axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]]
                imgs = []
                quiver_plots = [None] * 4
                robot_patches = []
                for ax, title, cmap in zip(ax_list, titles, cmaps):
                    ax.set_facecolor("#0d0d1a")
                    ax.set_title(title, color="white", fontsize=11)
                    ax.tick_params(colors="gray", labelsize=7)
                    img = ax.imshow(np.zeros((10, 10)), cmap=cmap, origin="lower", aspect="auto")
                    imgs.append(img)
                    cb = fig.colorbar(img, ax=ax, fraction=0.046, pad=0.04)
                    cb.ax.tick_params(colors="gray", labelsize=7)
                fig.tight_layout(rect=(0, 0, 1, 0.95))
                is_headless = True
            except Exception as e2:
                print(f"[CostmapVisualizer] Failed to initialize fallback costmap: {e2}")
                self._alive = False
                return

        try:
            while self._alive and (is_headless or plt.fignum_exists(fig.number)):
                with self._lock:
                    dirty = self._dirty
                    if dirty:
                        d = {k: v for k, v in self._data.items()}
                        self._dirty = False

                if dirty:
                    grids = [d["occupancy"], d["sdf"], d["grad_mag"], d["cost"]]
                    vmaxes = [1.0, d["sdf_max"], d["grad_mag"].max() + 1e-9, 1.0]

                    for img, grid, vmax in zip(imgs, grids, vmaxes):
                        img.set_data(grid)
                        img.set_clim(0, vmax)
                        img.set_extent([
                            d["map_origin_x"],
                            d["map_origin_x"] + grid.shape[1] * d["map_res"],
                            d["map_origin_y"],
                            d["map_origin_y"] + grid.shape[0] * d["map_res"],
                        ])

                    # Gradient quiver on cost panel
                    if quiver_plots[3] is not None:
                        try:
                            quiver_plots[3].remove()
                        except Exception:
                            pass
                    stride = max(1, int(0.5 / d["map_res"]))
                    gx = d["grad_x"][::stride, ::stride]
                    gy = d["grad_y"][::stride, ::stride]
                    rows_q = np.arange(0, d["grad_x"].shape[0], stride)
                    cols_q = np.arange(0, d["grad_x"].shape[1], stride)
                    xq = cols_q * d["map_res"] + d["map_origin_x"]
                    yq = rows_q * d["map_res"] + d["map_origin_y"]
                    Xq, Yq = np.meshgrid(xq, yq)
                    quiver_plots[3] = ax_list[3].quiver(
                        Xq, Yq, gx, gy,
                        color="white", alpha=0.25,
                        scale=None, scale_units="xy",
                        width=0.002,
                    )

                    # Clear previous robot markers
                    for p in robot_patches:
                        try:
                            p.remove()
                        except Exception:
                            pass
                    robot_patches.clear()

                    # Add robot circle and heading arrow to all subplots
                    rx = d["robot_col"] * d["map_res"] + d["map_origin_x"]
                    ry = d["robot_row"] * d["map_res"] + d["map_origin_y"]
                    r = self.cfg.vis_robot_size_m
                    heading = d["robot_heading"]
                    arrow_dx = np.cos(heading) * r * 2.5
                    arrow_dy = np.sin(heading) * r * 2.5

                    for ax in ax_list:
                        circ = mpatches.Circle(
                            (rx, ry), radius=r,
                            linewidth=1.5, edgecolor="cyan",
                            facecolor="none", zorder=5,
                        )
                        arr = mpatches.FancyArrow(
                            rx, ry, arrow_dx, arrow_dy,
                            width=r * 0.3,
                            head_width=r * 0.6,
                            head_length=r * 0.4,
                            color="yellow", zorder=6,
                        )
                        ax.add_patch(circ)
                        ax.add_patch(arr)
                        robot_patches.extend([circ, arr])

                    # Always save latest snapshot to costmap_latest.png
                    try:
                        fig.savefig("costmap_latest.png", dpi=100)
                    except Exception:
                        pass

                    if not is_headless:
                        fig.canvas.draw_idle()

                if is_headless:
                    time.sleep(0.1)
                else:
                    plt.pause(0.05)
        except Exception as e:
            print(f"[CostmapVisualizer] 2D render loop stopped: {e}")
        finally:
            try:
                plt.close(fig)
            except Exception:
                pass
            self._alive = False
