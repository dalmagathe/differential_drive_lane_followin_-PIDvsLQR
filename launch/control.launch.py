import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    pkg_desc = get_package_share_directory('duckiebot')
    lqr_params = os.path.join(pkg_desc, 'config', 'lqr_params.yaml')
    pid_params = os.path.join(pkg_desc, 'config', 'pid_params.yaml')
    robot_params = os.path.join(pkg_desc, 'config', 'robot_params.yaml')
    scenario_path = os.path.join(pkg_desc, 'config', 'scenario.yaml')

    with open(scenario_path, 'r') as f:
        pose = yaml.safe_load(f)['initial_pose']

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='lqr',
        description="Contrôleur à lancer : 'lqr' ou 'pid'",
    )
    controller = LaunchConfiguration('controller')

    path_error = Node(
        package='duckiebot',
        executable='path_error_node',
        parameters=[{
            'use_sim_time': True,
            'x0': float(pose['x']),
            'y0': float(pose['y']),
            'theta0': float(pose['theta']),
        }],
    )

    lqr = TimerAction(
        period=10.0,
        actions=[
            Node(
            package='duckiebot',
            executable='lqr_controller_node',
            parameters=[lqr_params, robot_params, {'use_sim_time': True},{'timeout': 10.0}],
            condition=LaunchConfigurationEquals('controller', 'lqr')
        )]
    )

    pid = TimerAction(
        period=10.0,
        actions=[
            Node(
        package='duckiebot',
        executable='pid_controller_node',
        parameters=[pid_params, robot_params, {'use_sim_time': True},{'timeout': 10.0}],
        condition=LaunchConfigurationEquals('controller', 'pid')
        )]
    )

    motor = Node(
        package='duckiebot',
        executable='motor_node.py',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([controller_arg, path_error, lqr, pid, motor])