import rclpy
from rclpy.node import Node

from PIL import Image

from nvblox_msgs.msg import DistanceMapSlice
from sensor_msgs.msg import PointCloud2, PointField

# TODO: if numpy is too laggy, use cupy, the cuda accelerated version
# Doesn't seem like its a problem so far though.
import numpy as np

class combine_esdf(Node):

    def __init__(self):
        super().__init__('combine_esdf')
        self.costmap_publisher = self.create_publisher(DistanceMapSlice, '/combined_esdf', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/combined_esdf_pointcloud', 10)

        self.above_ground_subscriber = self.create_subscription(
            DistanceMapSlice,
            '/nvblox_node/static_map_slice',
            self.above_ground_callback,
            10)
        
        self.below_ground_subscriber = self.create_subscription(
            DistanceMapSlice,
            '/nvblox_node/static_map_slice2',
            self.below_ground_callback,
            10)
        
        self.above_ground_pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/nvblox_node/static_esdf_pointcloud',
            self.above_ground_pointcloud_callback,
            10)
        
        self.below_ground_pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/nvblox_node/static_esdf_pointcloud2',
            self.below_ground_pointcloud_callback,
            10)

        # Could eventually add more below ground costmaps here
        self.above_ground_costmap = None
        self.below_ground_costmap_one = None
        self.below_origin = None


        self.above_ground_pointcloud = None
        self.below_ground_pointcloud = None

    def sigmoid(self, x):
        return 2 / (1 + np.exp((x*-10) + 10))


    def above_ground_callback(self, msg):
        if (len(msg.data) == 0):
            return
        if (self.below_ground_costmap_one is None):
            self.costmap_publisher.publish(msg)
            return
        self.above_ground_costmap = np.array(msg.data).reshape(msg.height, msg.width)
        # print(self.above_ground_costmap.shape, msg.origin, msg.resolution)
        
        
        # self.above_ground_costmap = np.array(msg.data).reshape(msg.height, msg.width)
        # # Set any null values in the costmap to an inaccessible area (before inverting)
        
        # condition2 = self.below_ground_costmap_one == 0
        # self.below_ground_costmap_one[condition2] = 1000
        condition = self.below_ground_costmap_one != 1000
        self.below_ground_costmap_one[condition] = 0

        origin = [msg.origin.x, msg.origin.y]

        print(origin, self.below_origin)

        # self.above_ground_costmap[]
        if (self.above_ground_costmap.shape[0] > self.below_ground_costmap_one.shape[0] and self.above_ground_costmap.shape[1] > self.below_ground_costmap_one.shape[1]):
            try:
                offset_x1 = abs(int((origin[0] - self.below_origin[0]) / msg.resolution))
                offset_y1 = abs(int((origin[1] - self.below_origin[1]) / msg.resolution))
                offset_x2 = ((self.above_ground_costmap.shape[0] - self.below_ground_costmap_one.shape[0]) - offset_x1)
                offset_y2 = ((self.above_ground_costmap.shape[1] - self.below_ground_costmap_one.shape[1]) - offset_y1)
                # print(offset_x1, offset_x2, self.above_ground_costmap.shape, self.below_ground_costmap_one.shape)
                # print(offset_x1, offset_x2, offset_y1, offset_y2)
                # self.above_ground_costmap[offset_x2:-offset_x1, offset_y2:-offset_y1] = 1000
                self.above_ground_costmap[offset_x2:-offset_x1, offset_y2:-offset_y1] = self.below_ground_costmap_one # np.minimum((self.above_ground_costmap[offset_x2:-offset_x1, offset_y2:-offset_y1]), self.below_ground_costmap_one)
                print(self.above_ground_costmap.dtype)

                # image2 = Image.fromarray(self.below_ground_costmap_one.astype('uint8'))
                # image2.save("numpy_matrix_image2.png")

                # image = Image.fromarray(self.above_ground_costmap.astype('uint8'))
                # image.save("numpy_matrix_image.png")
                #
                msg.data = self.above_ground_costmap.flatten().tolist()
                self.costmap_publisher.publish(msg)
                print("worked")
            except:
                print("did not work")
        else:
            pass
        if (self.above_ground_costmap.size < self.below_ground_costmap_one.size):
            print("shit")



    # Easily replicable for more below ground costmaps. Stack the costmaps into a 3D array
    # Probably useful to move the inversion logic to this function if this is the case.
    def below_ground_callback(self, msg):
        if (len(msg.data) == 0):
            return   
        self.below_ground_costmap_one = np.array(msg.data).reshape(msg.height, msg.width)
        self.below_origin = [msg.origin.x, msg.origin.y]




    # POINTCLOUD COMBINING LOGIC
    def above_ground_pointcloud_callback(self, msg):
        if (len(msg.data) == 0 or self.below_ground_pointcloud is None):
            return
        above_ground_float_values = np.frombuffer(msg.data, dtype=np.uint8).view(dtype=np.float32)
        below_ground_cloud = self.below_ground_pointcloud.view(dtype=np.float32)

        slice_indices = np.arange(0, len(above_ground_float_values), 4)
        slice_indices_below = np.arange(0, len(below_ground_cloud), 4)
        # Compare the coordinates:
        x_above_ground = above_ground_float_values[slice_indices]
        y_above_ground = above_ground_float_values[slice_indices + 1]
        data_above_ground = above_ground_float_values[slice_indices + 3]
        above_coords = np.array([x_above_ground, y_above_ground]).T

        x_below_ground = below_ground_cloud[slice_indices_below]
        y_below_ground = below_ground_cloud[slice_indices_below + 1]
        data_below_ground = below_ground_cloud[slice_indices_below + 3]

        
        data_below_ground[data_below_ground == 1000] = 0  # Add 1 to non-zero points
        data_below_ground = (2 - data_below_ground)  # Invert the esdf
        

        below_coords = np.array([x_below_ground, y_below_ground]).T # Merge the x and y coordinates into a 2D array -> [x, y]

        data_indices = np.where(np.all(above_coords[:,None] == below_coords[None, :], axis=-1)) # Coordinates where each point in the above ground cloud matches a point in the below ground cloud

        data_above_ground[data_indices[0]] = np.minimum(data_above_ground[data_indices[0]], data_below_ground[data_indices[1]])
        above_ground_float_values[slice_indices+3] = data_above_ground
        
        
        
        # above_ground_float_values[data_indices[0]] = np.maximum(above_ground_float_values[data_indices[0]], below_ground_cloud[data_indices[1]])

        message = msg
        message.data = np.frombuffer(above_ground_float_values.tobytes(), dtype=np.uint8).tolist()
        self.pointcloud_publisher.publish(message)


    def below_ground_pointcloud_callback(self, msg):
        if (len(msg.data) == 0):
            return
        self.below_ground_pointcloud = np.frombuffer(msg.data, dtype=np.uint8)

def main(args=None):
    rclpy.init(args=args)
    # print("hello world")
    node = combine_esdf()
    node.get_logger().info("Combine ESDF node started")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()














    # print(np.average(self.below_ground_costmap_one), np.amin(self.below_ground_costmap_one), np.amax(self.below_ground_costmap_one))
        # inverse_below_ground_costmap = np.full(self.below_ground_costmap_one.shape, 2) - self.below_ground_costmap_one

        # # pad the below ground costmap, to give it the same dimensions as the above ground costmap        
        # max_width = max(self.above_ground_costmap.shape[0], inverse_below_ground_costmap.shape[0])
        # max_height = max(self.above_ground_costmap.shape[1], inverse_below_ground_costmap.shape[1])
        # inverse_below_ground_costmap = np.pad(inverse_below_ground_costmap, ((max_width - inverse_below_ground_costmap.shape[0], 0), (max_height - inverse_below_ground_costmap.shape[1], 0)), 'constant', constant_values=(2))
        # self.above_ground_costmap = np.pad(self.above_ground_costmap, ((max_width - self.above_ground_costmap.shape[0], 0), (max_height - self.above_ground_costmap.shape[1], 0)), 'constant', constant_values=(2))

        # # Create a message to publish the combined costmap
        # message = DistanceMapSlice()
        # message = msg # should be largely identical to the original message.
        # if self.above_ground_costmap.shape == inverse_below_ground_costmap.shape:
        #     inverse_below_ground_costmap = inverse_below_ground_costmap * 1000
        #     message.data = np.maximum(inverse_below_ground_costmap, self.above_ground_costmap).flatten().tolist()
        #     self.costmap_publisher.publish(message)