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
    slam_share = get_package_share_directory('active_3d_slam')
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'nav2_explore.launch.py')
        ),
        launch_arguments={
            'startup_delay': '10.0',
            'cmd_vel_topic': '/model/tilted_turtlebot/cmd_vel',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/tilted_turtlebot/cmd_vel'
            '@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/tilted_turtlebot/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    tilted_scan_node = Node(
        package='active_3d_core',
        executable='tilted_scan_node',
        name='tilted_scan_node',
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
            'tf_timeout_sec': 0.3,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    octomap_cloud_node = Node(
        package='active_3d_core',
        executable='tilted_scan_node',
        name='octomap_cloud_node',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'pointcloud_topic': '/octomap_cloud_in',
            'target_frame': 'odom',
            'source_frame': 'base_scan',
            'use_latest_tf': True,
            'filter_ground': True,
            'ground_min_z': 0.08,
            'accumulate_cloud': False,
            'tf_timeout_sec': 0.3,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'odom',
            'resolution': 0.05,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('cloud_in', '/octomap_cloud_in'),
        ],
    )

    frontier_extractor_node = Node(
        package='active_3d_slam',
        executable='frontier_extractor_node',
        name='frontier_extractor_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('frontiers')),
        parameters=[{
            'octomap_topic': '/octomap_full',
            'traversability_grid_topic': '/traversability_grid',
            'frontier_marker_topic': '/frontier_voxels',
            'cluster_marker_topic': '/frontier_clusters',
            'candidate_pose_topic': '/frontier_candidate_poses',
            'scan_ready_topic': '/frontier_scan_ready',
            'best_goal_topic': '/best_frontier_goal',
            'best_goal_marker_topic': '/best_frontier_goal_marker',
            'cmd_vel_topic': '/model/tilted_turtlebot/cmd_vel',
            'floor_z': 0.0,
            'floor_clearance': 0.10,
            'robot_radius': 0.18,
            'robot_height': 0.35,
            'min_obstacle_z': 0.10,
            'max_free_z': 2.0,
            'grid_resolution': 0.05,
            'map_padding': 0.5,
            'min_cluster_size': 3,
            'max_candidate_count': 5,
            'robot_frame': 'base_footprint',
            'information_gain_weight': 1.0,
            'distance_weight': 0.35,
            'auto_send_nav2_goal': True,
            'use_visual_goal_selector': LaunchConfiguration('visual_goal_selector'),
            'selected_goal_topic': '/visual_selected_frontier_goal',
            'nav2_action_name': 'navigate_to_pose',
            'goal_update_period_sec': 8.0,
            'goal_replan_distance': 0.35,
            'initial_spin_enabled': True,
            'initial_spin_start_delay': 3.0,
            'initial_spin_angle': 6.283185307179586,
            'initial_spin_angular_speed': 0.45,
            'post_spin_settle_sec': 1.5,
            'stop_after_first_goal': False,
            'goal_arrival_tolerance': 0.14,
            'best_unknown_gain_threshold': 12.0,
            'low_voxel_gain_threshold': 8,
            'max_exploration_time_sec': 300.0,
            'max_goal_count': 8,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    visual_goal_selector_node = Node(
        package='active_3d_core',
        executable='visual_goal_selector_node',
        name='visual_goal_selector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('visual_goal_selector')),
        parameters=[{
            'candidate_pose_topic': '/frontier_candidate_poses',
            'scan_ready_topic': '/frontier_scan_ready',
            'image_topic': '/camera/image_raw',
            'selected_goal_topic': '/visual_selected_frontier_goal',
            'llm_request_debug_topic': '/llm_request_debug',
            'llm_response_debug_topic': '/llm_response_debug',
            'llm_exchange_debug_topic': '/llm_exchange_debug',
            'cmd_vel_topic': '/model/tilted_turtlebot/cmd_vel',
            'fixed_frame': 'odom',
            'robot_frame': 'base_footprint',
            'max_candidates': 5,
            'min_candidate_distance': LaunchConfiguration('min_candidate_distance'),
            'capture_only': LaunchConfiguration('visual_capture_only'),
            'capture_once': LaunchConfiguration('visual_capture_once'),
            'call_llm_api': LaunchConfiguration('call_llm_api'),
            'require_llm_response': LaunchConfiguration('require_llm_response'),
            'disable_llm_reasoning': LaunchConfiguration('disable_llm_reasoning'),
            'debug_image_dir': LaunchConfiguration('debug_image_dir'),
            'llm_api_url': LaunchConfiguration('llm_api_url'),
            'llm_model': LaunchConfiguration('llm_model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    cmd_vel_watch_node = Node(
        package='active_3d_core',
        executable='cmd_vel_watch_node',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/model/tilted_turtlebot/cmd_vel',
            'log_period_sec': 1.0,
            'deadband': 0.005,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
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
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time for every ROS node.',
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
            default_value='false',
            description='Start the WASD keyboard drive node.',
        ),
        DeclareLaunchArgument(
            'pointcloud_frame',
            default_value='odom',
            description='Frame used for /tilted_pointcloud.',
        ),
        DeclareLaunchArgument(
            'frontiers',
            default_value='true',
            description='Extract frontier voxels and clustered candidate poses.',
        ),
        DeclareLaunchArgument(
            'nav2',
            default_value='true',
            description='Start the minimal Nav2 stack used by frontier exploration.',
        ),
        DeclareLaunchArgument(
            'visual_goal_selector',
            default_value='true',
            description='Use camera snapshots and a local vision LLM to choose the next goal.',
        ),
        DeclareLaunchArgument(
            'visual_capture_only',
            default_value='false',
            description='Only capture and save candidate images without publishing a selected goal.',
        ),
        DeclareLaunchArgument(
            'visual_capture_once',
            default_value='true',
            description='Capture candidate images once, then ignore later candidate updates.',
        ),
        DeclareLaunchArgument(
            'call_llm_api',
            default_value='true',
            description='Call the local vision LLM API after candidate images are captured.',
        ),
        DeclareLaunchArgument(
            'require_llm_response',
            default_value='true',
            description='Do not publish a selected goal unless the LLM returns selected_index.',
        ),
        DeclareLaunchArgument(
            'disable_llm_reasoning',
            default_value='true',
            description='Ask the local LLM to disable reasoning/thinking output.',
        ),
        DeclareLaunchArgument(
            'min_candidate_distance',
            default_value='0.65',
            description='Ignore visual goal candidates closer than this distance from the robot.',
        ),
        DeclareLaunchArgument(
            'debug_image_dir',
            default_value='/tmp/active_3d_visual_goal_selector',
            description='Directory where captured candidate images are saved.',
        ),
        DeclareLaunchArgument(
            'llm_api_url',
            default_value='http://localhost:11434/api/chat',
            description='Ollama native local vision LLM chat endpoint.',
        ),
        DeclareLaunchArgument(
            'llm_model',
            default_value='gemma4:26b',
            description='Local vision model name passed to the LLM API.',
        ),
        gz_sim,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        scan_bridge,
        keyboard_drive_node,
        cmd_vel_watch_node,
        odom_tf_node,
        tilted_scan_node,
        octomap_cloud_node,
        octomap_server_node,
        frontier_extractor_node,
        visual_goal_selector_node,
        nav2,
        rviz,
    ])
