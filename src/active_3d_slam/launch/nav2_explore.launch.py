"""Launch the minimal Nav2 stack for the active 3D exploration demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start Nav2 servers that provide the /navigate_to_pose action."""
    slam_share = get_package_share_directory('active_3d_slam')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    log_level = LaunchConfiguration('log_level')
    startup_delay = LaunchConfiguration('startup_delay')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    params_path = os.path.join(slam_share, 'config', 'nav2_explore_params.yaml')
    common_args = ['--ros-args', '--log-level', log_level]
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    ament_prefix_path = os.pathsep.join(
        path for path in ['/opt/ros/jazzy', os.environ.get('AMENT_PREFIX_PATH', '')] if path
    )
    ld_library_path = os.pathsep.join(
        path for path in ['/opt/ros/jazzy/lib', os.environ.get('LD_LIBRARY_PATH', '')] if path
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'AMENT_PREFIX_PATH',
            ament_prefix_path,
        ),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            ld_library_path,
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_path,
            description='Nav2 parameters for the active 3D exploration demo.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/model/tilted_turtlebot/cmd_vel',
            description='Velocity command topic consumed by Gazebo.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 log level.',
        ),
        DeclareLaunchArgument(
            'startup_delay',
            default_value='8.0',
            description='Delay Nav2 lifecycle activation until odom TF is available.',
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='both',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=common_args,
            remappings=remappings + [('cmd_vel', cmd_vel_topic)],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='both',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=common_args,
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='both',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=common_args,
            remappings=remappings + [('cmd_vel', cmd_vel_topic)],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='both',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=common_args,
            remappings=remappings,
        ),
        TimerAction(
            period=startup_delay,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='both',
                    emulate_tty=True,
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': lifecycle_nodes,
                    }],
                    arguments=common_args,
                ),
            ],
        ),
    ])
