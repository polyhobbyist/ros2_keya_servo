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
            output='screen'),
        Node(
            package='ros2_socketcan',
            executable='ros2_socketcan',
            name='ros2_socketcan',
            output='screen',
            parameters=[{
                'interface': 'can0',
                'baudrate': 250000,
                'frame_id': 'can0',
                'receive_own_messages': False,
            }]),
    ])