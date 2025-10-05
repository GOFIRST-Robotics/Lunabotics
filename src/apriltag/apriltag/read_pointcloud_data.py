import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt

data = np.load('point_cloud_data.npz', allow_pickle=True)


# point_cloud = pv.PolyData(np.array([[0, 0, 0], [2, 2, 2]])) # data['points'][0:5])

points = np.array(data['points'])
# points = points[0:-1:2]

point_cloud = pv.PolyData(points)

mesh = point_cloud.reconstruct_surface()

plotter = pv.Plotter()

# plotter.add_mesh(point_cloud.point_normals, color='red', point_size=5, render_points_as_spheres=True)
plotter.add_mesh(mesh, color='white', opacity=0.5)
# plotter.add_mesh(point_cloud, color='white', point_size=10, render_points_as_spheres=True)
plotter.set_background('white')
plotter.show_grid()
plotter.show()