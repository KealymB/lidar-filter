# Lidar filter assignment 

This ROS2 package is a filter for Lidar data. Where the input is pointcloud2 messages and the output is filtered pointcloud2 messages. 

Filter requirements:
- Intensities under 1000 are filtered out
- Every X messages are removed

