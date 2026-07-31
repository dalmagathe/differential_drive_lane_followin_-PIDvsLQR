import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory('duckiebot')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    xacro_path = os.path.join(pkg_desc, 'urdf', 'duckiebot.xacro')
    scenario_path = os.path.join(pkg_desc, 'config', 'scenario.yaml')

    with open(scenario_path, 'r') as f:
        pose = yaml.safe_load(f)['initial_pose']

    j_mult = LaunchConfiguration('J_mult')
    declare_j_mult = DeclareLaunchArgument(
        'J_mult', default_value='1.0',
        description="Multiplicateur de l'inertie roue iyy dans l'URDF (plant seulement)"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        )
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                # l'espace avant 'J_mult' est obligatoire (sépare du chemin xacro)
                Command(['xacro ', xacro_path, ' J_mult:=', j_mult]),
                value_type=str
            ),
            'use_sim_time': True,
        }]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'duckiebot',
            '-timeout', '60',
            '-x', str(pose['x']),
            '-y', str(pose['y']),
            '-Y', str(pose['theta']),
        ]
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    wheel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_effort_controller'],
    )

    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
    )
    wheel_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[wheel_spawner])
    )

    return LaunchDescription([
        declare_j_mult,
        gazebo, rsp, spawn,
        jsb_after_spawn, wheel_after_jsb,
    ])