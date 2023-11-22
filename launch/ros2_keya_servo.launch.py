from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():


    return LaunchDescription([
        Node(
            package='ros2_keya_servo',
            executable='ros2_keya_servo',
            name='ros2_keya_servo',
            output='screen')
    ])