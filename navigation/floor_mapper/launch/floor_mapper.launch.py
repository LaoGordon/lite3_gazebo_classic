from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='floor_mapper',
            executable='floor_mapper_node',
            name='floor_mapper',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('floor_mapper'),
                    'config',
                    'floor_mapper.yaml',
                ]),
            ],
        )
    ])
