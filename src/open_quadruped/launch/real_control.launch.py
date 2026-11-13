"""Start the joystick-to-IK pipeline without starting a simulator.

A ROS 2 hardware driver must subscribe to /joint_angles or the trajectory
bridge output. The original ROS 1 rosserial/Teensy transport was not included
in the supplied source and is intentionally not emulated here.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        Node(
            package='joy', executable='joy_node', name='joy_node', output='screen',
            parameters=[{'device_id': ParameterValue(LaunchConfiguration('joy_device_id'), value_type=int), 'deadzone': 0.05}],
        ),
        Node(
            package='open_quadruped', executable='interface_process.py',
            name='interface_process', output='screen',
        ),
    ])
