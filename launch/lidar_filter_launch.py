import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lidar_filter_node = Node(
        package='lidar_filter_assignment',
        executable='lidar_filter',
        name='lidar_filter',
        output='screen',
        parameters=[
            {'intensity_threshold': 1000},
            {'message_throttle': 10}
        ]
    )

    dummy_lidar_publisher_node = Node(
        package='lidar_filter_assignment',
        executable='dummy_lidar_publisher',
        name='dummy_lidar_publisher',
        output='screen',
        parameters=[
            {'coordinate_deviation_in_meters': 3.0},
            {'number_of_points': 100}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('lidar_filter_assignment'), 'rviz_config.rviz')]
    )

    return LaunchDescription([
        lidar_filter_node,
        dummy_lidar_publisher_node,
        rviz_node
    ])
