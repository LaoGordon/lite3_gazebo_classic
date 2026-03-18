from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='floor_mapper',
            executable='floor_mapper_node',
            name='floor_mapper',
            output='screen',
            parameters=[
                {'input_topic': '/obstacle_points'},
                {'map_topic': '/floor_map'},
                {'scan_topic': '/scan'},
            ],
        )
    ])
