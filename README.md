# Lidar filter assignment

This ROS2 package is a filter for Lidar data. Where the input is pointcloud2 messages and the output is filtered pointcloud2 messages.

Filter requirements:

- Intensities under 1000 are filtered out
- Every X messages are removed

## Usage

To build the package use `colcon build`

Run the package by first sourcing it `source install/setup.bash`

Then launch it using `ros2 launch lidar_filter_assignment lidar_filter_launch.py`

You can also provide your own lidar data, by publishing it on the `/lidar_topic` and removing the `dummy_lidar_publisher_node` inside of the launch configuration
