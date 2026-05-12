"""Launch Gazebo, tilted scan projection, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Start a complete tilted lidar visualization stack."""
    description_share = get_package_share_directory('turtlebot3_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(
        description_share,
        'urdf',
        'turtlebot3_burger.urdf',
    )
    world_path = os.path.join(
        description_share,
        'worlds',
        'tilted_lidar_demo.sdf',
    )
    rviz_path = os.path.join(
        description_share,
        'rviz',
        'tilted_lidar.rviz',
    )

    robot_description = xacro.process_file(urdf_path).toxml()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{
            'name': 'tilted_turtlebot',
            'topic': '/robot_description',
            'x': 1.1,
            'y': 0.0,
            'z': 0.02,
        }],
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/tilted_turtlebot/cmd_vel'
            '@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/tilted_turtlebot/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        parameters=[{
            'qos_overrides./model/tilted_turtlebot/cmd_vel'
            '.subscriber.reliability': 'reliable',
        }],
    )

    odom_tf_node = Node(
        package='active_3d_core',
        executable='odom_tf_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
        parameters=[{
            'odom_topic': '/model/tilted_turtlebot/odometry',
            'parent_frame': 'odom',
            'child_frame': 'base_footprint',
            'stamp_with_current_time': True,
            'initial_x': 1.1,
            'initial_y': 0.0,
            'initial_z': 0.0,
            'use_sim_time': False,
        }],
    )

    tilted_scan_node = Node(
        package='active_3d_core',
        executable='tilted_scan_node',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'pointcloud_topic': '/tilted_pointcloud',
            'target_frame': LaunchConfiguration('pointcloud_frame'),
            'source_frame': 'base_scan',
            'use_latest_tf': True,
            'filter_ground': True,
            'ground_min_z': 0.05,
            'accumulate_cloud': True,
            'max_accumulated_points': 200000,
            'use_sim_time': False,
        }],
    )

    keyboard_drive_node = Node(
        package='active_3d_core',
        executable='keyboard_drive_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('keyboard')),
        parameters=[{
            'cmd_vel_topic': '/model/tilted_turtlebot/cmd_vel',
            'linear_speed': 0.16,
            'angular_speed': 0.7,
            'spin_angular_speed': 0.45,
            'key_timeout': 0.3,
            'publish_cmd_vel': True,
            'publish_tf': False,
            'parent_frame': 'odom',
            'child_frame': 'base_footprint',
            'initial_x': 1.1,
            'initial_y': 0.0,
            'initial_z': 0.0,
            'initial_yaw': 0.0,
            'use_sim_time': False,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(description_share),
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz with the tilted point cloud display.',
        ),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description=(
                'Publish odom -> base_footprint from Gazebo odometry. '
                'Keep true so RViz follows the real Gazebo robot pose.'
            ),
        ),
        DeclareLaunchArgument(
            'keyboard',
            default_value='true',
            description='Start the WASD keyboard drive node.',
        ),
        DeclareLaunchArgument(
            'pointcloud_frame',
            default_value='odom',
            description='Frame used for /tilted_pointcloud.',
        ),
        gz_sim,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        scan_bridge,
        keyboard_drive_node,
        odom_tf_node,
        tilted_scan_node,
        rviz,
    ])
