from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('bfmc_sign_detection'), 'config', 'awareness.yaml'
    ])
    return LaunchDescription([
        Node(
            package='bfmc_sign_detection',
            executable='road_sign_detector',
            name='road_sign_detector',
            output='screen',
            parameters=[config],
        )
    ])
