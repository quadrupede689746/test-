"""Start Gazebo and the bridge for a terminal-driven crawl gait coordinator."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


BENT_POSE = [
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
]
BENT_POSE_CSV = ','.join(str(value) for value in BENT_POSE)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('open_quadruped'), 'launch', 'sim.launch.py'
            ])),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'joystick_control': 'false',
                'spawn_z': '0.245',
                'gravity_settle_delay_sec': '20.0',
                'initial_joint_positions': BENT_POSE_CSV,
                'place_initial_stance': 'true',
            }.items(),
        ),
        Node(
            package='open_quadruped',
            executable='joint_angles_to_trajectory.py',
            name='joint_angles_to_trajectory',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'initial_joint_positions': BENT_POSE,
                'startup_duration_sec': 10.0,
            }],
        ),
    ])
