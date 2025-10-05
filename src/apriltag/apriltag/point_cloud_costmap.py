import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
import time
from scipy.ndimage import gaussian_filter, gaussian_laplace
from skimage.filters import threshold_otsu
from skimage.feature import blob_log


# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from sensor_msgs.msg import PointCloud2

start = time.time()
data = np.load('point_cloud_data.npz', allow_pickle=True)

points = np.array(data['points'])


def project_point_cloud_optimized(points, resolution=0.01):
    """
    Projects a 3D point cloud onto a 2D heightmap using a highly optimized
    NumPy approach. This version avoids explicit sorting and Python loops for
    aggregation, resulting in a significant performance increase.

    Args:
        points (np.ndarray): A NumPy array of shape (N, 3) representing the point cloud.
        resolution (float): The resolution of the projected 2D grid.
        min_height (float): The minimum height to initialize the grid. This value
                            will also be used to identify empty cells, which are
                            later converted to NaN.

    Returns:
        np.ndarray: A 2D NumPy array representing the heightmap.
    """
    world_min_y = np.min(points[:, 1])
    world_max_y = np.max(points[:, 1])
    world_min_x = np.min(points[:, 0])
    world_max_x = np.max(points[:, 0])

    grid_x_size = int((world_max_x - world_min_x) / resolution) + 1
    grid_y_size = int((world_max_y - world_min_y) / resolution) + 1
    
    projected = np.full((grid_x_size, grid_y_size), fill_value=np.nan, dtype=np.float32)
    projected_x_indices = ((points[:, 0] - world_min_x) / resolution).astype(np.int32)
    projected_y_indices = ((points[:, 1] - world_min_y) / resolution).astype(np.int32)
    sums = np.zeros_like(projected)
    counts = np.zeros_like(projected, dtype=int)

    np.add.at(sums, (projected_x_indices, projected_y_indices), points[:, 2])
    np.add.at(counts, (projected_x_indices, projected_y_indices), 1)

    average_heightmap = np.divide(sums, counts, out=np.full_like(sums, np.nan), where=counts!=0)
    projected = average_heightmap

    origin = (world_min_x, world_min_y)
    return projected, origin

# def isolate_brightest_clusters(image, min_size=0.2):
#     thresh = threshold_otsu(image)
#     binary = image > thresh

#     from scipy.ndimage import label, find_objects

#     labeled, num_features = label(binary)
#     slices = find_objects(labeled)

#     output = np.zeros_like(image)

#     for i, slc in enumerate(slices):
#         region = labeled[slc] == (i + 1)
#         if np.sum(region) >= min_size:
#             output[slc][region] = image[slc][region]

#     return output

def isolate_brightest_clusters(image, min_size=0.00):
    blobs = blob_log(image, min_sigma=.6, max_sigma=1.4, num_sigma=10, threshold=0.03)

    output = np.zeros_like(image)

    for blob in blobs:
        y, x, r = blob
        y = int(y)
        x = int(x)
        r = int(r)
        if r >= min_size:
            output[y, x] = image[y, x]

    # output = gaussian_laplace(image, sigma=1)
    return output

def generate_costmap(points, resolution=0.01, unknown_cost=255, lethal_slope_degrees=10, lethal_cost=254):
    lethal_slope = np.tan(np.deg2rad(lethal_slope_degrees))
    projected, origin = project_point_cloud_optimized(points, resolution=resolution)

    # lpg = gaussian_laplace(projected, sigma=1)
    # return lpg
    # plt.imshow(projected)
    # plt.show()

    # smoothed_projected = gaussian_filter(projected, sigma=0)
    gradient = np.gradient(projected, resolution, edge_order=1)
    magnitude = np.sqrt(gradient[0]**2 + gradient[1]**2)

    is_nan_mask = np.isnan(magnitude)
    mean_height = np.nanmean(magnitude)
    magnitude = np.nan_to_num(magnitude, nan=mean_height)

    lpg = gaussian_laplace(magnitude, sigma=2)
    return lpg


    # gradient = np.gradient(magnitude, resolution, edge_order=1)
    # magnitude = np.sqrt(gradient[0]**2 + gradient[1]**2)

    brightest_only = isolate_brightest_clusters(magnitude)
    brightest_only[is_nan_mask] = np.nan

    brightest_only[brightest_only > 0] = lethal_cost
    brightest_only[np.isnan(brightest_only)] = 0
    return brightest_only
    magnitude = np.nan_to_num(magnitude, nan=unknown_cost)

    costmap = np.full_like(magnitude, fill_value=unknown_cost, dtype=np.uint8)
    costmap[magnitude <= lethal_slope] = 0
    costmap[(magnitude > lethal_slope) & (magnitude < 2 * lethal_slope)] = lethal_cost

    costmap[is_nan_mask] = unknown_cost

    return costmap


resolution = 0.1

# projected = project_point_cloud_optimized(points, resolution=resolution)
costmap = generate_costmap(points, resolution=resolution)

end = time.time()
print(f"Time taken: {end - start} seconds")
plt.imshow(costmap)
plt.show()

point_cloud = pv.PolyData(points)
world_min_y = np.min(points[:, 1])
world_max_y = np.max(points[:, 1])
world_min_x = np.min(points[:, 0])
world_max_x = np.max(points[:, 0])

    

plotter = pv.Plotter()
scalars = costmap[((points[:, 0] - world_min_x) // resolution).astype(int), ((points[:, 1] - world_min_y) // resolution).astype(int)]
plotter.add_mesh(point_cloud, scalars=scalars, point_size=15, render_points_as_spheres=True)
plotter.set_background('white')
plotter.show()