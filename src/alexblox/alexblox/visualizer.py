import rclpy
from rclpy.node import Node
import numpy as np
import pyvista as pv  # this isnt in the container right now,
# either pip install each time, or rebuild the container. i added it as a dependency.

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from threading import Lock


class CostmapVisualizer(Node):
    def __init__(self):
        super().__init__("costmap_visualizer")
        self.lock = Lock()

        self.declare_parameter("costmap_resolution", 0.1)
        self.resolution = float(self.get_parameter("costmap_resolution").value)

        self.declare_parameter("lethal_slope", 0.3)
        self.lethal_slope = float(self.get_parameter("lethal_slope").value)
        # who knows what units this is in

        self.declare_parameter("lethal_cost", 254)
        self.lethal_cost = int(self.get_parameter("lethal_cost").value)

        self.declare_parameter("unknown_cost", 255)
        self.unknown_cost = int(self.get_parameter("unknown_cost").value)

        self.declare_parameter(
            "input_topic", "/zed2i/zed_node_zed2i/mapping/fused_cloud"
        )
        input_topic = str(self.get_parameter("input_topic").value)

        self.declare_parameter("output_topic", "/costmap/costmap")
        output_topic = str(self.get_parameter("output_topic").value)

        self.pointCloudSubscription = self.create_subscription(
            PointCloud2, input_topic, self.pointCloudCallback, 10
        )

        self.costmapPublisher = self.create_publisher(OccupancyGrid, output_topic, 10)
        # self.distanceMapSubscription = self.create_subscription(
        #     DistanceMapSlice,
        #     '/nvblox_node/static_map_slice',
        #     self.distanceMapCallback,
        #     10
        # )
        self.get_logger().info(f"Costmap generating on {input_topic} -> {output_topic}")
        print(input_topic, output_topic)

    def pointCloudCallback(self, msg):
        if len(msg.data) == 0:
            return

        with self.lock:
            print("Received PointCloud2 message")
            resolution = self.resolution

            # UNPACK THE POINT CLOUD DATA
            data = np.frombuffer(msg.data, dtype=np.uint8).view(dtype=np.float32)
            points = data.reshape(-1, 4)[:, :3]

            world_min_y = np.min(points[:, 1])
            world_max_y = np.max(points[:, 1])
            world_min_x = np.min(points[:, 0])
            world_max_x = np.max(points[:, 0])
            grid_x_size = int((world_max_x - world_min_x) / resolution) + 1
            grid_y_size = int((world_max_y - world_min_y) / resolution) + 1

            # ESDF PROJECTION
            projected = np.full(
                (grid_x_size, grid_y_size), fill_value=np.nan, dtype=np.float32
            )
            projected_x_indices = ((points[:, 0] - world_min_x) / resolution).astype(
                np.int32
            )
            projected_y_indices = ((points[:, 1] - world_min_y) / resolution).astype(
                np.int32
            )
            sums = np.zeros_like(projected)
            counts = np.zeros_like(projected, dtype=int)

            np.add.at(sums, (projected_x_indices, projected_y_indices), points[:, 2])
            np.add.at(counts, (projected_x_indices, projected_y_indices), 1)

            average_heightmap = np.divide(
                sums, counts, out=np.full_like(sums, np.nan), where=counts != 0
            )
            projected = average_heightmap

            # origin = (world_min_x, world_min_y)

            # EDGE DETECTION
            gradient = np.gradient(projected, resolution, edge_order=1)
            magnitude = np.sqrt(gradient[0] ** 2 + gradient[1] ** 2)
            costmap = np.full_like(
                magnitude, fill_value=self.unknown_cost, dtype=np.uint8
            )
            costmap[magnitude <= self.lethal_slope] = 0
            costmap[magnitude > self.lethal_slope] = self.lethal_cost

            point_cloud = pv.PolyData(points)
            plotter = pv.Plotter()
            scalars = costmap[
                ((points[:, 0] - world_min_x) // resolution).astype(int),
                ((points[:, 1] - world_min_y) // resolution).astype(int),
            ]
            plotter.add_mesh(
                point_cloud,
                scalars=scalars,
                point_size=15,
                render_points_as_spheres=True,
            )
            plotter.set_background("white")
            plotter.show()


def main(args=None):
    rclpy.init(args=args)

    costmap_visualizer = CostmapVisualizer()

    rclpy.spin(costmap_visualizer)

    costmap_visualizer.destroy_node()
    rclpy.shutdown()
