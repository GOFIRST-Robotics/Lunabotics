# this is an implementation of the full update, point method for point processing: https://www.notion.so/Ground-Algorithm-ab538c4bde484908abdde515edaa24a0?pvs=4
# by Eric Patton (with code from others)
# last updated 4/24/2023

import sys
import os

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs

import numpy as np
import open3d as o3d

#define constants
CELL_SIZE = 10 #size of each cell in mm
CELL_POINTS_CUTOFF = 5 # minimum number of points required for a cell to be considered statistically valuable\
GRAD_THRESH = 1 # gradient threshold to mark a cell as dangerous, always positive, 1 = 45 degrees

class Cell():
    def __init__(self):
        points = []
        danger = 0 # 0 is no danger, 1 is lacking info, 2 is high slope
        avg = None

class PointcloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        #
        self.get_logger().info('Node trying to start')#debug to see if starting correctly
        #
        ## This is for visualization of the received point cloud.
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = o3d.geometry.PointCloud()
        #
        self.subscription = self.create_subscription(
            sensor_msgs.PointCloud2,
            'points',
            self.listener_callback,
            10
        )
        self.subscription #prevent unused variable warning
        #
        self.get_logger().info('Node started successfully')# isn't running
    #
    def listener_callback(self, msg):
        #
        allPoints = np.array(list(read_points(msg)))
        #self.get_logger().info(f'shape: {allPoints.shape}')
        #
        sortedPoints, minx, miny = self.points_to_cells(allPoints)
        #
        self.flag_cells(sortedPoints)
        #
        # The rest here is for visualization. (modified from https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_subscriber/pcd_subscriber_node.py)
        self.vis.remove_geometry(self.o3d_pcd)
        self.o3d_pcd = o3d.geometry.PointCloud(
                            o3d.utility.Vector3dVector(allPoints))
        #
        self.color_points(self.o3d_pcd, minx, miny, sortedPoints)
        #
        self.vis.add_geometry(self.o3d_pcd)
        #
        self.vis.poll_events()
        self.vis.update_renderer()
    #
    def points_to_cells(self, allPoints):
        """convert list of points from pointcloud input into 2d array of lists of points"""
        #
        minx = maxx = allPoints[0][0]
        miny = maxy = allPoints[0][1]
        minz = maxz = allPoints[0][2]
        #
        for i in range(len(allPoints)):
            if allPoints[i][0] > maxx:
                maxx = allPoints[i][0]
            elif allPoints[i][0] < minx:
                minx = allPoints[i][0]
            #
            if allPoints[i][1] > maxy:
                maxy = allPoints[i][1]
            elif allPoints[i][1] < miny:
                miny = allPoints[i][1]
            #
            if allPoints[i][2] > maxz:
                maxz = allPoints[i][2]
            elif allPoints[i][2] < minz:
                minz = allPoints[i][2]
        #
        cell_num_x = int((maxx - minx) / CELL_SIZE)
        cell_num_y = int((maxy - miny) / CELL_SIZE)
        #cell_num_z = (maxz - minz) / CELL_SIZE
        #
        cells = np.empty((cell_num_x, cell_num_y), dtype=object) # create cells array
        for cellRow in cells:
            for cell in cellRow:
                cell = Cell()
        #
        for point in allPoints: #organize all points into cells
            x_index = int(point[0] / CELL_SIZE - (minx / CELL_SIZE)) # use cell size and minimum to calc cell index to insert into
            y_index = int(point[1] / CELL_SIZE - (miny / CELL_SIZE))
            #
            cells[x_index][y_index].points.append(point) # TODO: thinks cell retrived is None
        #
        return cells, minx, miny
    #
    def flag_cells(self, sortedPoints):
        """Take cells of points and find the dangerous ones"""
        # the follow could be improved by screening out outliers in the cell
        for i, cellRow in enumerate(sortedPoints):
            for j, cell in enumerate(cellRow):
                if len(cell.points) < CELL_POINTS_CUTOFF: # determine if the cell has too few points to be valuable
                    cell.danger = 1
                    break
                else:
                    #
                    cell.avg = sum(v[2] for v in cell.points) / len(cell.points) # set average z value of the cell
                    #
                    if(j == 0):# only check gradient left
                        if(i != 0):# cell isn't first to be checked
                            x_grad = get_gradient(cell.avg, sortedPoints[i-1][0])
                            if(x_grad > GRAD_THRESH):
                                cell.danger = 2
                    else:
                        y_grad = get_gradient(cell.avg, sortedPoints[i][j-1]) #check top gradient
                        if(y_grad > GRAD_THRESH):
                            cell.danger = 2
                            break
                        if(i != 0):# check left gradient also
                            x_grad = get_gradient(cell.avg, sortedPoints[i-1][0])
                            if(x_grad > GRAD_THRESH):
                                cell.danger = 2
        # there is no return value because the function will affect the original cell array
    #
    def get_gradient(self, avg_a, avg_b):
        """get z gradient between two cells averages"""
        return abs((avg_b - avg_a) / CELL_SIZE) # every cell average is separated by the cell size (there are no diagonals)
    #
    def color_points(self, pointCloud, minx, miny, cells):
        pointCloud.paint_uniform_color([0, 255, 0]) # 0-255 rgb scale
        #
        for i, pointColor in enumerate(pointCloud.colors):
            point = pointCloud.points[i] # get point coordinates
            #
            x_index = int(point[0] / CELL_SIZE - (minx / CELL_SIZE))
            y_index = int(point[1] / CELL_SIZE - (miny / CELL_SIZE))
            #
            cell = cells[x_index][y_index].points.append(point)
            if(cell.danger == 1):
                pointColor = [1, 0, 0] # 0-1 rgb scale
            elif(cell.danger == 2):
                pointCloud = [0, 0, 1]
        # no return value because object is directly edited
        
        
## The code below is "ported" from 
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt



def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    points_node = PointcloudSubscriber()
    rclpy.spin(points_node)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    points_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()