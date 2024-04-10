import rclpy
from rclpy.node import Node

from scipy.ndimage import gaussian_filter
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
            1)
        
        self.below_ground_subscriber = self.create_subscription(
            DistanceMapSlice,
            '/nvblox_node/static_map_slice2',
            self.below_ground_callback,
            1)
        
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
        self.below_ground_costmap = None
        self.below_origin = None


        self.above_ground_pointcloud = None
        self.below_ground_pointcloud = None

    def sigmoid(self, x):
        return (2 / (1 + np.exp((x*-10) + 3)))


    def above_ground_callback(self, msg):
        if (self.below_ground_costmap is None):
            return
        below_origin = self.below_origin
        below_ground_costmap_one = np.copy(self.below_ground_costmap)
        if (len(msg.data) == 0):
            return
        if (below_ground_costmap_one is None):
            self.costmap_publisher.publish(msg)
            return
        self.above_ground_costmap = np.array(msg.data).reshape(msg.height, msg.width)
        
        
        
        
        # condition = below_ground_costmap_one != 1000
        # below_ground_costmap_one[condition] = 0

        origin = [msg.origin.x, msg.origin.y]

        # print(origin, self.below_origin)

        # self.above_ground_costmap[]
        if (self.above_ground_costmap.shape[0] > below_ground_costmap_one.shape[0] and self.above_ground_costmap.shape[1] > below_ground_costmap_one.shape[1]):
            try:
                print("worked")
                offset_x1 = abs(int((origin[1] - below_origin[1]) / msg.resolution))
                offset_y1 = abs(int((origin[0] - below_origin[0]) / msg.resolution))
                offset_x2 = ((self.above_ground_costmap.shape[0] - below_ground_costmap_one.shape[0]) - offset_x1)
                offset_y2 = ((self.above_ground_costmap.shape[1] - below_ground_costmap_one.shape[1]) - offset_y1)
                

                # To invert or not to invert, that is the question.
                # Comment this next chunk out if the lower esdf is only showing the hole as red, not the terrain.
                # below_ground_costmap_one[below_ground_costmap_one == 1000] = 0
                # below_ground_costmap_one = np.full(below_ground_costmap_one.shape, 2) - below_ground_costmap_one
                # below_ground_costmap_one = 2 - self.sigmoid(below_ground_costmap_one)


                # if you don't want to invert, uncomment this:
                condition2 = below_ground_costmap_one <=2
                below_ground_costmap_one[condition2] = 0


                # Actually combine the costmaps.
                self.above_ground_costmap[offset_x1:-offset_x2, offset_y1:-offset_y2] = np.minimum((self.above_ground_costmap[offset_x1:-offset_x2, offset_y1:-offset_y2]), below_ground_costmap_one)
        
                # Blur the costmap, so that the hole is less abrupt        
                self.above_ground_costmap = gaussian_filter(self.above_ground_costmap, sigma=5)


                msg.data = self.above_ground_costmap.flatten().tolist()
                self.costmap_publisher.publish(msg)
            except:
                print("did not work")
        else:
            pass
        if (self.above_ground_costmap.size < below_ground_costmap_one.size):
            print("shit")


    # Easily replicable for more below ground costmaps. Stack the costmaps into a 3D array
    # Probably useful to move the inversion logic to this function if this is the case.
    def below_ground_callback(self, msg):
        if (len(msg.data) == 0):
            return   
        self.below_ground_costmap = np.array(msg.data).reshape(msg.height, msg.width)
        self.below_origin = [msg.origin.x, msg.origin.y]
        # print(msg.resolution)



    # POINTCLOUD COMBINING LOGIC
    # Performance is garbage, inadvisable to run during comps
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

        
        data_below_ground[data_below_ground <=2] = 0  # Add 1 to non-zero points
        # data_below_ground = (2 - data_below_ground)  # Invert the esdf
        

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











# print(offset_x1, offset_x2, self.above_ground_costmap.shape, below_ground_costmap_one.shape)
                # print(offset_x1, offset_x2, offset_y1, offset_y2)
                
                # print(np.average(below_ground_costmap_one), np.amin(below_ground_costmap_one), np.amax(below_ground_costmap_one))
                
                # below_ground_costmap_one = np.flip(below_ground_costmap_one, axis=1)
                # self.above_ground_costmap[offset_x2:-offset_x1, offset_y1:-offset_y2] = 1000
            


    # print(np.average(below_ground_costmap_one), np.amin(below_ground_costmap_one), np.amax(below_ground_costmap_one))
        # inverse_below_ground_costmap = np.full(below_ground_costmap_one.shape, 2) - below_ground_costmap_one

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



         # self.above_ground_costmap[np.where(self.above_ground_costmap == 0)] = 1000
                
                # self.above_ground_costmap = np.flip(self.above_ground_costmap, axis=1)
                # self.above_ground_costmap[0:offset_x2, 0:5] = 0
                # self.above_ground_costmap[0:10, 0:offset_y1] = 0
                # self.above_ground_costmap[-offset_x1:-1, -10:] = 0
                # self.above_ground_costmap[-10:, -offset_y2:-1] = 0
                # image2 = Image.fromarray(below_ground_costmap_one.astype('uint8'))
                # image2.save("numpy_matrix_image2.png")

                # image = Image.fromarray(self.above_ground_costmap.astype('uint8'))
                # image.save("numpy_matrix_image.png")