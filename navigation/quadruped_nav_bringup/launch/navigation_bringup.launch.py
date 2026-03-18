from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fastlivo_nav_bridge'),
                'launch',
                'bridge.launch.py',
            ])
        )
    )

    floor_mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('floor_mapper'),
                'launch',
                'floor_mapper.launch.py',
            ])
        )
    )

    return LaunchDescription([
        bridge_launch,
        floor_mapper_launch,
    ])
