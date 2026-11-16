"""Start Gazebo with the keyboard-controlled crawl gait coordinator."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('step_length_m', default_value='0.04'),
        DeclareLaunchArgument('swing_height_m', default_value='0.025'),
        DeclareLaunchArgument('swing_duration_sec', default_value='1.0'),
        DeclareLaunchArgument('dwell_duration_sec', default_value='0.20'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('open_quadruped'), 'launch', 'sim.launch.py'
            ])),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'joystick_control': 'false',
            }.items(),
        ),
        Node(
            package='open_quadruped',
            executable='joint_angles_to_trajectory.py',
            name='joint_angles_to_trajectory',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='open_quadruped',
            executable='crawl_gait_coordinator.py',
            name='crawl_gait_coordinator',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': True,
                'step_length_m': LaunchConfiguration('step_length_m'),
                'swing_height_m': LaunchConfiguration('swing_height_m'),
                'swing_duration_sec': LaunchConfiguration('swing_duration_sec'),
                'dwell_duration_sec': LaunchConfiguration('dwell_duration_sec'),
            }],
        ),
    ])
