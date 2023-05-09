# Point Processing Explained

The goal of this node is to take pointcloud information and processing it into something useful for a pathfinder.

WARNING: This node is not currently functional.

## The method

The node receives the pointcloud in the form of a list of points.
Then, it will convert it into a grid of cells with an assigned danger level.
To do this, it uses the following steps:

1. Find the minimum and maximum of the x and y axes in order to determine the dimensions of the grid
2. Organize all the points in the point cloud by their position into the appropriate cell
3. Loop through every cell and mark cells with few points as dangerous
4. In the same loop find the average z value of the cell and use that calculate the gradient with nearby cells; if the gradient is too high it's marked as dangerous

## Usage details

Currently, the node is set up to receive test information from the test_publisher node.
This node publishes to the "points" topic.
It will accept any PointCloud2 message, and the topic can be changed on line 40 in "gen_grid_world.py" to do so.

### Running the node

The node can be compiled and run using the usual commands:

`colcon build --packages-select point_processing`

`ros2 run point_processing pointcloud_subscriber`

To run the test publisher node, a .ply sample file must be provided:

`ros2 run point_processing pointcloud_test_publisher "./Pointcloud Examples/airplane.ply"`

### Visualization

The processing node includes a segment of code for visualization.
The code is currently commented out, but can be uncommented and used without issues.
The lines in question are 62-73 in the "gen_grid_world.py".
It can be expected that the visualization will slow down the processing.

### Current bugs

Currently, the node is not functional.
The problem appears to be in the flag_cells, as cells are not being properly flagged as dangerous.
Please keep this in mind while attempting to use the node.