"""Start Gazebo for controlled bent-stance position-loop tests."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


POSE = '0,-0.68162889,1.26606748,0,-0.68162889,1.26606748,0,-0.68162889,1.26606748,0,-0.68162889,1.26606748'
POSE_VALUES = [
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('startup_duration_sec', default_value='10.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('open_quadruped'), 'launch', 'sim.launch.py'
            ])),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'joystick_control': 'false',
                'spawn_z': '0.245',
                'gravity_settle_delay_sec': '20.0',
                'initial_joint_positions': POSE,
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
                'initial_joint_positions': POSE_VALUES,
                'startup_duration_sec': LaunchConfiguration('startup_duration_sec'),
            }],
        ),
    ])
