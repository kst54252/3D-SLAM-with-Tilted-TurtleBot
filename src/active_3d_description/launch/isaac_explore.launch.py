"""Launch the exploration stack for an Isaac Sim-provided robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Start ROS-side mapping, frontier selection, Nav2, and RViz for Isaac Sim."""
    description_share = get_package_share_directory('turtlebot3_description')
    slam_share = get_package_share_directory('active_3d_slam')

    urdf_path = os.path.join(
        description_share,
        'urdf',
        'turtlebot3_burger.urdf',
    )
    rviz_path = os.path.join(
        description_share,
        'rviz',
        'tilted_lidar.rviz',
    )
    nav2_params_path = os.path.join(
        slam_share,
        'config',
        'nav2_isaac_params.yaml',
    )

    robot_description = xacro.process_file(urdf_path).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'nav2_explore.launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'startup_delay': LaunchConfiguration('nav2_startup_delay'),
            'cmd_vel_topic': cmd_vel_topic,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('robot_state_publisher')),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    odom_tf_node = Node(
        package='active_3d_core',
        executable='odom_tf_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
        parameters=[{
            'odom_topic': odom_topic,
            'parent_frame': odom_frame,
            'child_frame': base_frame,
            'stamp_with_current_time': True,
            'use_sim_time': use_sim_time,
        }],
    )

    tilted_scan_node = Node(
        package='active_3d_core',
        executable='tilted_scan_node',
        name='tilted_scan_node',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'pointcloud_topic': '/tilted_pointcloud',
            'target_frame': odom_frame,
            'source_frame': lidar_frame,
            'use_latest_tf': True,
            'filter_ground': True,
            'ground_min_z': 0.05,
            'accumulate_cloud': True,
            'max_accumulated_points': 200000,
            'use_sim_time': use_sim_time,
        }],
    )

    octomap_cloud_node = Node(
        package='active_3d_core',
        executable='tilted_scan_node',
        name='octomap_cloud_node',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'pointcloud_topic': '/octomap_cloud_in',
            'target_frame': odom_frame,
            'source_frame': lidar_frame,
            'use_latest_tf': True,
            'filter_ground': True,
            'ground_min_z': 0.08,
            'accumulate_cloud': False,
            'use_sim_time': use_sim_time,
        }],
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': odom_frame,
            'resolution': 0.05,
            'use_sim_time': use_sim_time,
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
            'cmd_vel_topic': cmd_vel_topic,
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
            'robot_frame': base_frame,
            'information_gain_weight': 1.0,
            'distance_weight': 0.35,
            'auto_send_nav2_goal': True,
            'use_visual_goal_selector': LaunchConfiguration('visual_goal_selector'),
            'selected_goal_topic': '/visual_selected_frontier_goal',
            'nav2_action_name': 'navigate_to_pose',
            'goal_update_period_sec': 8.0,
            'goal_replan_distance': 0.35,
            'initial_spin_enabled': LaunchConfiguration('initial_spin'),
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
            'use_sim_time': use_sim_time,
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
            'image_topic': LaunchConfiguration('image_topic'),
            'selected_goal_topic': '/visual_selected_frontier_goal',
            'llm_request_debug_topic': '/llm_request_debug',
            'llm_response_debug_topic': '/llm_response_debug',
            'llm_exchange_debug_topic': '/llm_exchange_debug',
            'cmd_vel_topic': cmd_vel_topic,
            'fixed_frame': odom_frame,
            'robot_frame': base_frame,
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
            'use_sim_time': use_sim_time,
        }],
    )

    cmd_vel_watch_node = Node(
        package='active_3d_core',
        executable='cmd_vel_watch_node',
        output='screen',
        parameters=[{
            'cmd_vel_topic': cmd_vel_topic,
            'log_period_sec': 1.0,
            'deadband': 0.005,
            'use_sim_time': use_sim_time,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Isaac Sim /clock.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity command topic consumed by Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic published by Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic published by Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='RGB camera image topic published by Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Global odometry frame used by Nav2 and OctoMap.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot base frame used by Nav2.',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='base_scan',
            description='Frame id of the Isaac Sim lidar scan.',
        ),
        DeclareLaunchArgument(
            'robot_state_publisher',
            default_value='false',
            description='Publish URDF TF locally if Isaac does not publish robot TF.',
        ),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='false',
            description='Publish odom -> base frame from /odom if Isaac does not publish TF.',
        ),
        DeclareLaunchArgument(
            'initial_spin',
            default_value='true',
            description='Rotate once before selecting the first frontier.',
        ),
        DeclareLaunchArgument(
            'frontiers',
            default_value='true',
            description='Extract frontier voxels and send one Nav2 goal.',
        ),
        DeclareLaunchArgument(
            'nav2',
            default_value='true',
            description='Start the minimal Nav2 stack.',
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
        DeclareLaunchArgument(
            'nav2_startup_delay',
            default_value='8.0',
            description='Delay Nav2 lifecycle activation until Isaac TF is available.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz.',
        ),
        robot_state_publisher,
        odom_tf_node,
        tilted_scan_node,
        octomap_cloud_node,
        octomap_server_node,
        frontier_extractor_node,
        visual_goal_selector_node,
        cmd_vel_watch_node,
        nav2,
        rviz,
    ])
