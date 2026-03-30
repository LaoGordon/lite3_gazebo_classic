from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fastlivo_nav_bridge',
            executable='fastlivo_nav_bridge_node',
            name='fastlivo_nav_bridge',
            output='screen',
            parameters=[
                {'fastlivo_odom_topic': '/aft_mapped_to_init'},
                {'fastlivo_cloud_topic': '/cloud_registered_lidar'},
                {'odom_topic': '/odom'},
                {'obstacle_topic': '/obstacle_points'},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_frame': 'base'},
                {'publish_identity_map_to_odom': True},
            ],
        )
    ])
