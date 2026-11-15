import os
import re
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # --------------------------------------------------
    # 1. Generate robot description using Xacro
    # --------------------------------------------------

    package_share = get_package_share_directory('open_quadruped')

    xacro_file = os.path.join(
        package_share,
        'urdf',
        'spot.urdf.xacro'
    )

    print('[OpenQuadruped] Generating robot description...')

    result = subprocess.run(
        ['xacro', xacro_file],
        capture_output=True,
        text=True,
        check=True
    )

    robot_xml = result.stdout

    # --------------------------------------------------
    # 2. Remove ALL XML comments
    # --------------------------------------------------

    robot_xml, removed_comments = re.subn(
        r'<!--.*?-->',
        '',
        robot_xml,
        flags=re.DOTALL
    )

    print(
        f'[OpenQuadruped] Removed {removed_comments} XML comments.'
    )

    # --------------------------------------------------
    # 3. Validate generated URDF
    # --------------------------------------------------

    ET.fromstring(robot_xml)

    print('[OpenQuadruped] URDF XML validation successful.')

    robot_description = ParameterValue(
        robot_xml,
        value_type=str
    )

    # --------------------------------------------------
    # 4. Robot State Publisher
    # --------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # --------------------------------------------------
    # 5. Spawn robot in Gazebo
    # --------------------------------------------------

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_open_quadruped',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'open_quadruped',
            '-x', '0.0',
            '-y', '0.0',
            # At the all-zero joint pose, FK puts the foot-sphere centers at
            # z=-0.2222 m relative to base_link; 0.245 m gives ~3 mm clearance.
            '-z', '0.245',
        ],
    )

    # --------------------------------------------------
    # 6. Joystick
    # --------------------------------------------------

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joystick_control')),
        parameters=[{
            'device_id': ParameterValue(
                LaunchConfiguration('joy_device_id'),
                value_type=int
            ),
            'deadzone': 0.05,
            'autorepeat_rate': 10.0,
        }],
    )

    # --------------------------------------------------
    # 7. Inverse Kinematics
    # --------------------------------------------------

    interface_node = Node(
        package='open_quadruped',
        executable='interface_process.py',
        name='interface_process',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joystick_control')),
        parameters=[{
            'publish_rate_hz': 10.0,
            'use_sim_time': True,
        }],
    )

    # --------------------------------------------------
    # 8. Joint Trajectory Bridge
    # --------------------------------------------------

    trajectory_bridge = Node(
        package='open_quadruped',
        executable='joint_angles_to_trajectory.py',
        name='joint_angles_to_trajectory',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joystick_control')),
        parameters=[{
            'use_sim_time': True,
        }],
    )



    # --------------------------------------------------
    # 9. Spawn controllers AFTER robot insertion
    # --------------------------------------------------

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', '--inactive',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )
    load_quadruped_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'quadruped_controller', '--inactive',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )
    unpause_world = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty', '{}'],
        output='screen',
    )
    activate_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', 'quadruped_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '15',
        ],
        output='screen',
    )
    enable_gravity = Node(
        package='open_quadruped',
        executable='enable_robot_gravity.py',
        name='enable_robot_gravity',
        output='screen',
        parameters=[{
            'model_name': 'open_quadruped',
            'settle_delay_sec': 1.0,
        }],
    )

    return [
        robot_state_publisher,
        spawn_robot,
        joy_node,
        interface_node,
        trajectory_bridge,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_quadruped_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_quadruped_controller,
                on_exit=[unpause_world],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=unpause_world,
                on_exit=[activate_controllers],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=activate_controllers,
                on_exit=[enable_gravity],
            )
        ),
    ]


def generate_launch_description():

    use_gui = LaunchConfiguration('gui')

    # --------------------------------------------------
    # Gazebo Classic.  The Humble gazebo_ros launch file shipped on this
    # machine constructs ``-s`` plugin arguments as one token (``-slib...``),
    # which prevents Gazebo from loading the ROS factory plugin.  Keep the
    # same official plugins, but pass each option and value as separate argv
    # entries so /spawn_entity is guaranteed to be created.
    gazebo_world = os.path.join(
        get_package_share_directory('open_quadruped'), 'worlds', 'crawl.world'
    )

    # Both gzserver and gzclient must be able to resolve model:// mesh URIs.
    # The parent of the package share directory contains the
    # open_quadruped/ folder expected by model://open_quadruped/...
    open_quadruped_share = get_package_share_directory('open_quadruped')
    gazebo_model_path = os.pathsep.join([
        '/usr/share/gazebo-11/models',
        os.path.dirname(open_quadruped_share),
    ])
    gazebo = ExecuteProcess(
        cmd=[
            'gzserver', '-u', gazebo_world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '--verbose',
        ],
        output='screen',
        additional_env={
            'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib',
            'GAZEBO_MODEL_PATH': gazebo_model_path,
            # Do not block startup on the retired public model database.
            'GAZEBO_MODEL_DATABASE_URI': '',
        },
        on_exit=Shutdown(),
    )
    gazebo_gui = ExecuteProcess(
        cmd=['gzclient'],
        condition=IfCondition(use_gui),
        output='screen',
        additional_env={
            # gzclient loads visual meshes locally; it needs the same model
            # search path as gzserver to resolve model:// URIs.
            'GAZEBO_MODEL_PATH': gazebo_model_path,
        },
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo Classic GUI.',
        ),
        DeclareLaunchArgument(
            'joystick_control',
            default_value='true',
            description='Start the joystick body-pose control pipeline.',
        ),

        DeclareLaunchArgument(
            'joy_device_id',
            default_value='0',
            description='Joystick device index.',
        ),

        gazebo,
        gazebo_gui,

        OpaqueFunction(
            function=launch_setup
        ),
    ])
