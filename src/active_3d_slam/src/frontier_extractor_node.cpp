#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef ACTIVE_3D_SLAM_HAS_NAV2
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#endif

namespace active_3d_slam
{
namespace
{

struct Grid2D
{
  double resolution;
  double origin_x;
  double origin_y;
  int width;
  int height;
  std::vector<bool> known_free;
  std::vector<bool> occupied;
  std::vector<bool> inflated_occupied;
  std::vector<bool> traversable;
  std::vector<bool> frontier;

  int index(int x, int y) const
  {
    return y * width + x;
  }

  bool inBounds(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < width && y < height;
  }

  geometry_msgs::msg::Point cellCenter(int x, int y, double z) const
  {
    geometry_msgs::msg::Point point;
    point.x = origin_x + (static_cast<double>(x) + 0.5) * resolution;
    point.y = origin_y + (static_cast<double>(y) + 0.5) * resolution;
    point.z = z;
    return point;
  }
};

struct Cluster
{
  geometry_msgs::msg::Point candidate;
  std::size_t size;
};

struct ScoredCandidate
{
  Cluster cluster;
  double information_gain;
  double distance;
  double score;
};

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace

class FrontierExtractorNode : public rclcpp::Node
{
public:
#ifdef ACTIVE_3D_SLAM_HAS_NAV2
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
#endif

  FrontierExtractorNode()
  : Node("frontier_extractor_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_full");
    traversability_grid_topic_ =
      declare_parameter<std::string>("traversability_grid_topic", "/traversability_grid");
    nav_obstacle_grid_topic_ =
      declare_parameter<std::string>("nav_obstacle_grid_topic", "/nav_obstacle_grid");
    frontier_marker_topic_ =
      declare_parameter<std::string>("frontier_marker_topic", "/frontier_voxels");
    cluster_marker_topic_ =
      declare_parameter<std::string>("cluster_marker_topic", "/frontier_clusters");
    candidate_pose_topic_ =
      declare_parameter<std::string>("candidate_pose_topic", "/frontier_candidate_poses");
    scan_ready_topic_ =
      declare_parameter<std::string>("scan_ready_topic", "/frontier_scan_ready");
    best_goal_topic_ =
      declare_parameter<std::string>("best_goal_topic", "/best_frontier_goal");
    best_goal_marker_topic_ =
      declare_parameter<std::string>("best_goal_marker_topic", "/best_frontier_goal_marker");
    selected_goal_topic_ =
      declare_parameter<std::string>("selected_goal_topic", "/visual_selected_frontier_goal");
    cmd_vel_topic_ = declare_parameter<std::string>(
      "cmd_vel_topic", "/model/tilted_turtlebot/cmd_vel");

    floor_z_ = declare_parameter<double>("floor_z", 0.0);
    floor_clearance_ = declare_parameter<double>("floor_clearance", 0.10);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.18);
    robot_height_ = declare_parameter<double>("robot_height", 0.35);
    min_obstacle_z_ = declare_parameter<double>("min_obstacle_z", floor_z_ + floor_clearance_);
    max_free_z_ = declare_parameter<double>("max_free_z", 2.0);
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.0);
    map_padding_ = declare_parameter<double>("map_padding", 0.5);
    min_cluster_size_ = declare_parameter<int>("min_cluster_size", 3);
    max_candidate_count_ = declare_parameter<int>("max_candidate_count", 5);
    max_grid_cells_ = declare_parameter<int>("max_grid_cells", 1000000);
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_footprint");
    information_gain_weight_ = declare_parameter<double>("information_gain_weight", 1.0);
    distance_weight_ = declare_parameter<double>("distance_weight", 0.35);
    auto_send_nav2_goal_ = declare_parameter<bool>("auto_send_nav2_goal", true);
    use_visual_goal_selector_ = declare_parameter<bool>("use_visual_goal_selector", false);
    nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
    goal_update_period_sec_ = declare_parameter<double>("goal_update_period_sec", 8.0);
    goal_replan_distance_ = declare_parameter<double>("goal_replan_distance", 0.35);
    stop_after_first_goal_ = declare_parameter<bool>("stop_after_first_goal", false);
    goal_arrival_tolerance_ = declare_parameter<double>("goal_arrival_tolerance", 0.14);
    use_distance_goal_monitor_ = declare_parameter<bool>("use_distance_goal_monitor", true);
    goal_arrival_stable_count_required_ =
      declare_parameter<int>("goal_arrival_stable_count_required", 8);
    min_goal_active_sec_ = declare_parameter<double>("min_goal_active_sec", 2.0);
    best_unknown_gain_threshold_ = declare_parameter<double>("best_unknown_gain_threshold", 12.0);
    low_voxel_gain_threshold_ = declare_parameter<int>("low_voxel_gain_threshold", 8);
    max_exploration_time_sec_ = declare_parameter<double>("max_exploration_time_sec", 300.0);
    max_goal_count_ = declare_parameter<int>("max_goal_count", 8);
    initial_spin_enabled_ = declare_parameter<bool>("initial_spin_enabled", true);
    initial_spin_start_delay_ = declare_parameter<double>("initial_spin_start_delay", 3.0);
    initial_spin_angle_ = declare_parameter<double>("initial_spin_angle", 6.283185307179586);
    initial_spin_angular_speed_ = declare_parameter<double>("initial_spin_angular_speed", 0.45);
    post_spin_settle_sec_ = declare_parameter<double>("post_spin_settle_sec", 1.5);
    frontier_alpha_ = declare_parameter<double>("frontier_alpha", 0.75);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    traversability_grid_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(
      traversability_grid_topic_, rclcpp::QoS(1).transient_local());
    nav_obstacle_grid_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(
      nav_obstacle_grid_topic_, rclcpp::QoS(1).transient_local());
    frontier_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(frontier_marker_topic_, 1);
    cluster_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(cluster_marker_topic_, 1);
    candidate_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>(candidate_pose_topic_, 1);
    scan_ready_pub_ =
      create_publisher<std_msgs::msg::Bool>(scan_ready_topic_, rclcpp::QoS(1).transient_local());
    best_goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(best_goal_topic_, 1);
    best_goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(best_goal_marker_topic_, 1);

#ifdef ACTIVE_3D_SLAM_HAS_NAV2
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
#endif

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&FrontierExtractorNode::octomapCallback, this, std::placeholders::_1));
    selected_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      selected_goal_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&FrontierExtractorNode::selectedGoalCallback, this, std::placeholders::_1));
    controller_transition_sub_ = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/controller_server/transition_event", 10,
      [this](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
        controller_active_ =
          msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        RCLCPP_INFO(
          get_logger(), "controller_server lifecycle state: %s",
          msg->goal_state.label.c_str());
      });
    bt_transition_sub_ = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/bt_navigator/transition_event", 10,
      [this](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
        bt_navigator_active_ =
          msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        RCLCPP_INFO(
          get_logger(), "bt_navigator lifecycle state: %s",
          msg->goal_state.label.c_str());
      });
    initial_spin_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FrontierExtractorNode::initialSpinTimerCallback, this));
    goal_monitor_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&FrontierExtractorNode::goalMonitorTimerCallback, this));

    if (!initial_spin_enabled_) {
      initial_spin_complete_ = true;
      publishScanReady(true);
    } else {
      publishScanReady(false);
    }

    RCLCPP_INFO(
      get_logger(),
      "Exploration pipeline: initial 360 scan -> OctoMap -> scored frontier goal -> Nav2 action");
  }

private:
  void initialSpinTimerCallback()
  {
    if (!initial_spin_enabled_ || initial_spin_complete_ || exploration_complete_) {
      return;
    }

    const auto now = get_clock()->now();
    if (!initial_spin_started_) {
      initial_spin_started_ = true;
      initial_spin_start_time_ = now;
      publishScanReady(false);
      if (exploration_start_time_.nanoseconds() == 0) {
        exploration_start_time_ = now;
      }
      RCLCPP_INFO(
        get_logger(),
        "Scan rotation will start in %.1f s", initial_spin_start_delay_);
    }

    const double elapsed = (now - initial_spin_start_time_).seconds();
    if (elapsed < initial_spin_start_delay_) {
      publishVelocity(0.0, 0.0);
      return;
    }

    const double angular_speed = std::max(0.01, std::abs(initial_spin_angular_speed_));
    const double spin_duration = std::abs(initial_spin_angle_) / angular_speed;
    if (elapsed < initial_spin_start_delay_ + spin_duration) {
      publishVelocity(0.0, angular_speed);
      return;
    }

    publishVelocity(0.0, 0.0);
    ++initial_spin_stop_count_;
    if (initial_spin_stop_count_ >= 10) {
      initial_spin_complete_ = true;
      initial_spin_complete_time_ = now;
      recordCompletedScanRotation();
      publishScanReady(true);
      RCLCPP_INFO(get_logger(), "360 scan complete; selecting a frontier goal.");
    }
  }

  void publishScanReady(bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    scan_ready_pub_->publish(message);
  }

  void publishVelocity(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear_x;
    command.angular.z = angular_z;
    cmd_vel_pub_->publish(command);
  }

  void goalMonitorTimerCallback()
  {
    if (
      !use_distance_goal_monitor_ || !goal_active_ || !has_last_goal_ ||
      exploration_complete_)
    {
      return;
    }

    const std::string goal_frame = last_goal_.header.frame_id.empty() ?
      std::string("odom") : last_goal_.header.frame_id;
    const auto robot_position = lookupRobotPosition(goal_frame);
    if (!robot_position) {
      return;
    }

    const double distance_to_goal = distance2D(*robot_position, last_goal_.pose.position);
    if (distance_to_goal > goal_arrival_tolerance_) {
      goal_arrival_stable_count_ = 0;
      return;
    }

    const auto now = get_clock()->now();
    if (last_goal_time_.nanoseconds() > 0 &&
      (now - last_goal_time_).seconds() < min_goal_active_sec_)
    {
      goal_arrival_stable_count_ = 0;
      return;
    }

    ++goal_arrival_stable_count_;
    if (goal_arrival_stable_count_ < goal_arrival_stable_count_required_) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Robot stayed within %.2f m of the frontier goal for %d samples; starting the next scan rotation.",
      goal_arrival_tolerance_, goal_arrival_stable_count_);
    handleGoalReached(true);
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    const std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
      octomap_msgs::fullMsgToMap(*msg));
    if (!abstract_tree) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Could not deserialize OctoMap. Use /octomap_full, not /octomap_binary.");
      return;
    }

    const auto * tree = dynamic_cast<const octomap::OcTree *>(abstract_tree.get());
    if (!tree) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "2D traversability currently supports octomap::OcTree messages only.");
      return;
    }

    latest_voxel_count_ = countLeafVoxels(*tree);

    auto grid = createGrid(*tree);
    if (grid.width == 0 || grid.height == 0) {
      return;
    }

    fillKnownFreeAndOccupied(*tree, grid);
    inflateOccupied(grid);
    computeTraversableAndFrontier(grid);
    const auto robot_position = lookupRobotPosition(msg->header.frame_id);
    const std::vector<Cluster> clusters = clusterFrontierCells(grid, robot_position);
    const auto scored_candidates = scoreCandidates(*msg, clusters);
    const auto candidates = limitCandidates(scored_candidates);
    const auto best_goal = candidates.empty() ?
      std::optional<ScoredCandidate>{} : std::optional<ScoredCandidate>{candidates.front()};

    publishTraversabilityGrid(*msg, grid);
    publishNavObstacleGrid(*msg, grid);
    publishFrontiers(*msg, grid);
    publishClusters(*msg, grid, candidates);
    publishCandidatePoses(*msg, candidates);
    if (use_visual_goal_selector_) {
      if (has_visual_selected_goal_) {
        publishVisualSelectedBestGoal(visual_selected_goal_);
      } else {
        publishBestGoal(*msg, grid, std::optional<ScoredCandidate>{});
      }
    } else {
      publishBestGoal(*msg, grid, best_goal);
    }
    updateExplorationStopConditions(best_goal, candidates.size());
    maybeSendNav2Goal(*msg, best_goal);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "2D grid: %dx%d, frontier cells: %zu, candidates: %zu%s",
      grid.width, grid.height, countTrue(grid.frontier), candidates.size(),
      best_goal ? ", best goal selected" : "");
  }

  void selectedGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!use_visual_goal_selector_ || !auto_send_nav2_goal_) {
      return;
    }
    if (exploration_complete_ || goal_active_) {
      return;
    }
    if (!initial_spin_complete_) {
      return;
    }

    const auto now = get_clock()->now();
    if (initial_spin_enabled_ &&
      (now - initial_spin_complete_time_).seconds() < post_spin_settle_sec_)
    {
      return;
    }
    if (last_goal_time_.nanoseconds() > 0 &&
      (now - last_goal_time_).seconds() < goal_update_period_sec_)
    {
      return;
    }

    auto goal_pose = *msg;
    if (goal_pose.header.frame_id.empty()) {
      goal_pose.header.frame_id = "odom";
    }
    if (goal_pose.header.stamp.sec == 0 && goal_pose.header.stamp.nanosec == 0) {
      goal_pose.header.stamp = now;
    }
    if (has_last_goal_ &&
      distance2D(goal_pose.pose.position, last_goal_.pose.position) < goal_replan_distance_)
    {
      return;
    }

    visual_selected_goal_ = goal_pose;
    has_visual_selected_goal_ = true;
    publishVisualSelectedBestGoal(goal_pose);
    sendNav2Goal(goal_pose, "visual selector", 0.0, 0.0, 0.0);
  }

  Grid2D createGrid(const octomap::OcTree & tree) const
  {
    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);

    const double resolution = grid_resolution_ > 0.0 ? grid_resolution_ : tree.getResolution();
    const double origin_x = std::floor((min_x - map_padding_) / resolution) * resolution;
    const double origin_y = std::floor((min_y - map_padding_) / resolution) * resolution;
    const double end_x = std::ceil((max_x + map_padding_) / resolution) * resolution;
    const double end_y = std::ceil((max_y + map_padding_) / resolution) * resolution;
    const int width = static_cast<int>(std::ceil((end_x - origin_x) / resolution));
    const int height = static_cast<int>(std::ceil((end_y - origin_y) / resolution));
    const auto cell_count = static_cast<std::int64_t>(width) * static_cast<std::int64_t>(height);

    Grid2D grid;
    if (width <= 0 || height <= 0 || cell_count > max_grid_cells_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping traversability grid with invalid/large size: %dx%d cells", width, height);
      return grid;
    }

    grid.resolution = resolution;
    grid.origin_x = origin_x;
    grid.origin_y = origin_y;
    grid.width = width;
    grid.height = height;
    grid.known_free.assign(cell_count, false);
    grid.occupied.assign(cell_count, false);
    grid.inflated_occupied.assign(cell_count, false);
    grid.traversable.assign(cell_count, false);
    grid.frontier.assign(cell_count, false);
    return grid;
  }

  void fillKnownFreeAndOccupied(const octomap::OcTree & tree, Grid2D & grid) const
  {
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
      const auto center = it.getCoordinate();
      const double z = center.z();
      const bool occupied = tree.isNodeOccupied(*it);

      if (occupied) {
        if (z < min_obstacle_z_ || z > floor_z_ + robot_height_) {
          continue;
        }
      } else if (z <= floor_z_ + floor_clearance_ || z > max_free_z_) {
        continue;
      }

      markLeafCells(grid, center.x(), center.y(), it.getSize(), occupied);
    }
  }

  void markLeafCells(
    Grid2D & grid, double center_x, double center_y, double leaf_size, bool occupied) const
  {
    const double half_size = std::max(leaf_size * 0.5, grid.resolution * 0.5);
    const int min_x = worldToCellX(grid, center_x - half_size);
    const int max_x = worldToCellX(grid, center_x + half_size);
    const int min_y = worldToCellY(grid, center_y - half_size);
    const int max_y = worldToCellY(grid, center_y + half_size);

    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        if (!grid.inBounds(x, y)) {
          continue;
        }
        const int index = grid.index(x, y);
        if (occupied) {
          grid.occupied[index] = true;
        } else {
          grid.known_free[index] = true;
        }
      }
    }
  }

  void inflateOccupied(Grid2D & grid) const
  {
    grid.inflated_occupied = grid.occupied;
    const int radius_cells = static_cast<int>(std::ceil(robot_radius_ / grid.resolution));
    const double radius_sq = robot_radius_ * robot_radius_;

    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        if (!grid.occupied[grid.index(x, y)]) {
          continue;
        }

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (!grid.inBounds(nx, ny)) {
              continue;
            }
            const double distance_sq =
              static_cast<double>(dx * dx + dy * dy) * grid.resolution * grid.resolution;
            if (distance_sq <= radius_sq) {
              grid.inflated_occupied[grid.index(nx, ny)] = true;
            }
          }
        }
      }
    }
  }

  void computeTraversableAndFrontier(Grid2D & grid) const
  {
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const int index = grid.index(x, y);
        grid.traversable[index] = grid.known_free[index] && !grid.inflated_occupied[index];
      }
    }

    constexpr int neighbor_offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const int index = grid.index(x, y);
        if (!grid.traversable[index]) {
          continue;
        }

        for (const auto & offset : neighbor_offsets) {
          const int nx = x + offset[0];
          const int ny = y + offset[1];
          if (!grid.inBounds(nx, ny)) {
            continue;
          }
          const int neighbor_index = grid.index(nx, ny);
          const bool unknown =
            !grid.known_free[neighbor_index] && !grid.occupied[neighbor_index];
          if (unknown) {
            grid.frontier[index] = true;
            break;
          }
        }
      }
    }
  }

  std::vector<Cluster> clusterFrontierCells(
    const Grid2D & grid,
    const std::optional<geometry_msgs::msg::Point> & robot_position) const
  {
    const auto reachable = computeReachableTraversableCells(grid, robot_position);
    std::vector<bool> visited(grid.frontier.size(), false);
    std::vector<Cluster> clusters;
    constexpr int neighbor_offsets[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };

    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const int start_index = grid.index(x, y);
        if (!grid.frontier[start_index] || !reachable[start_index] || visited[start_index]) {
          continue;
        }

        std::queue<std::pair<int, int>> queue;
        std::vector<std::pair<int, int>> cells;
        queue.push({x, y});
        visited[start_index] = true;

        while (!queue.empty()) {
          const auto [cx, cy] = queue.front();
          queue.pop();
          cells.push_back({cx, cy});

          for (const auto & offset : neighbor_offsets) {
            const int nx = cx + offset[0];
            const int ny = cy + offset[1];
            if (!grid.inBounds(nx, ny)) {
              continue;
            }
            const int neighbor_index = grid.index(nx, ny);
            if (!grid.frontier[neighbor_index] || !reachable[neighbor_index] ||
              visited[neighbor_index])
            {
              continue;
            }
            visited[neighbor_index] = true;
            queue.push({nx, ny});
          }
        }

        if (cells.size() < static_cast<std::size_t>(std::max(1, min_cluster_size_))) {
          continue;
        }

        clusters.push_back(makeCluster(grid, cells));
      }
    }

    std::sort(
      clusters.begin(), clusters.end(),
      [](const Cluster & a, const Cluster & b) {return a.size > b.size;});
    return clusters;
  }

  std::vector<bool> computeReachableTraversableCells(
    const Grid2D & grid,
    const std::optional<geometry_msgs::msg::Point> & robot_position) const
  {
    std::vector<bool> reachable(grid.traversable.size(), false);
    if (!robot_position) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping frontier candidates because robot pose is unavailable for reachability check.");
      return reachable;
    }

    const auto start = findNearestTraversableCell(grid, *robot_position);
    if (!start) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping frontier candidates because no traversable cell is near the robot.");
      return reachable;
    }

    constexpr int neighbor_offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    std::queue<std::pair<int, int>> queue;
    queue.push(*start);
    reachable[grid.index(start->first, start->second)] = true;

    while (!queue.empty()) {
      const auto [cx, cy] = queue.front();
      queue.pop();

      for (const auto & offset : neighbor_offsets) {
        const int nx = cx + offset[0];
        const int ny = cy + offset[1];
        if (!grid.inBounds(nx, ny)) {
          continue;
        }
        const int neighbor_index = grid.index(nx, ny);
        if (!grid.traversable[neighbor_index] || reachable[neighbor_index]) {
          continue;
        }
        reachable[neighbor_index] = true;
        queue.push({nx, ny});
      }
    }

    return reachable;
  }

  std::optional<std::pair<int, int>> findNearestTraversableCell(
    const Grid2D & grid,
    const geometry_msgs::msg::Point & robot_position) const
  {
    const int robot_x = worldToCellX(grid, robot_position.x);
    const int robot_y = worldToCellY(grid, robot_position.y);
    if (grid.inBounds(robot_x, robot_y) && grid.traversable[grid.index(robot_x, robot_y)]) {
      return std::make_pair(robot_x, robot_y);
    }

    const int search_radius =
      std::max(1, static_cast<int>(std::ceil((robot_radius_ * 2.0) / grid.resolution)));
    double best_distance_sq = std::numeric_limits<double>::max();
    std::optional<std::pair<int, int>> best_cell;

    for (int dy = -search_radius; dy <= search_radius; ++dy) {
      for (int dx = -search_radius; dx <= search_radius; ++dx) {
        const int x = robot_x + dx;
        const int y = robot_y + dy;
        if (!grid.inBounds(x, y) || !grid.traversable[grid.index(x, y)]) {
          continue;
        }
        const auto center = grid.cellCenter(x, y, floor_z_);
        const double cx = center.x - robot_position.x;
        const double cy = center.y - robot_position.y;
        const double distance_sq = cx * cx + cy * cy;
        if (distance_sq < best_distance_sq) {
          best_distance_sq = distance_sq;
          best_cell = std::make_pair(x, y);
        }
      }
    }

    return best_cell;
  }

  Cluster makeCluster(
    const Grid2D & grid,
    const std::vector<std::pair<int, int>> & cells) const
  {
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto & [x, y] : cells) {
      const auto point = grid.cellCenter(x, y, floor_z_);
      mean_x += point.x;
      mean_y += point.y;
    }
    mean_x /= static_cast<double>(cells.size());
    mean_y /= static_cast<double>(cells.size());

    double best_distance_sq = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point best_point;
    for (const auto & [x, y] : cells) {
      const auto point = grid.cellCenter(x, y, floor_z_);
      const double dx = point.x - mean_x;
      const double dy = point.y - mean_y;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_point = point;
      }
    }

    Cluster cluster;
    cluster.candidate = best_point;
    cluster.size = cells.size();
    return cluster;
  }

  std::vector<ScoredCandidate> scoreCandidates(
    const octomap_msgs::msg::Octomap & msg,
    const std::vector<Cluster> & clusters)
  {
    const auto robot_position = lookupRobotPosition(msg.header.frame_id);
    std::vector<ScoredCandidate> scored;
    scored.reserve(clusters.size());

    for (const auto & cluster : clusters) {
      ScoredCandidate candidate;
      candidate.cluster = cluster;
      candidate.information_gain = static_cast<double>(cluster.size);
      candidate.distance = robot_position ?
        distance2D(cluster.candidate, *robot_position) : 0.0;
      candidate.score =
        information_gain_weight_ * std::sqrt(candidate.information_gain) -
        distance_weight_ * candidate.distance;
      scored.push_back(candidate);
    }

    std::sort(
      scored.begin(), scored.end(),
      [](const ScoredCandidate & a, const ScoredCandidate & b) {
        return a.score > b.score;
      });
    return scored;
  }

  std::optional<geometry_msgs::msg::Point> lookupRobotPosition(const std::string & frame_id)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        frame_id, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      geometry_msgs::msg::Point point;
      point.x = transform.transform.translation.x;
      point.y = transform.transform.translation.y;
      point.z = transform.transform.translation.z;
      return point;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot score goal distance without TF %s -> %s: %s",
        frame_id.c_str(), robot_frame_.c_str(), exc.what());
      return std::nullopt;
    }
  }

  double distance2D(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b) const
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  std::vector<ScoredCandidate> limitCandidates(
    const std::vector<ScoredCandidate> & candidates) const
  {
    if (max_candidate_count_ <= 0 ||
      candidates.size() <= static_cast<std::size_t>(max_candidate_count_))
    {
      return candidates;
    }
    return std::vector<ScoredCandidate>(
      candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(max_candidate_count_));
  }

  void publishTraversabilityGrid(
    const octomap_msgs::msg::Octomap & msg,
    const Grid2D & grid)
  {
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header = msg.header;
    occupancy_grid.info.resolution = static_cast<float>(grid.resolution);
    occupancy_grid.info.width = static_cast<std::uint32_t>(grid.width);
    occupancy_grid.info.height = static_cast<std::uint32_t>(grid.height);
    occupancy_grid.info.origin.position.x = grid.origin_x;
    occupancy_grid.info.origin.position.y = grid.origin_y;
    occupancy_grid.info.origin.position.z = floor_z_;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.data.assign(grid.width * grid.height, -1);

    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const int index = grid.index(x, y);
        if (grid.inflated_occupied[index]) {
          occupancy_grid.data[index] = 100;
        } else if (grid.traversable[index]) {
          occupancy_grid.data[index] = 0;
        }
      }
    }

    traversability_grid_pub_->publish(occupancy_grid);
  }

  void publishNavObstacleGrid(
    const octomap_msgs::msg::Octomap & msg,
    const Grid2D & grid)
  {
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header = msg.header;
    occupancy_grid.info.resolution = static_cast<float>(grid.resolution);
    occupancy_grid.info.width = static_cast<std::uint32_t>(grid.width);
    occupancy_grid.info.height = static_cast<std::uint32_t>(grid.height);
    occupancy_grid.info.origin.position.x = grid.origin_x;
    occupancy_grid.info.origin.position.y = grid.origin_y;
    occupancy_grid.info.origin.position.z = floor_z_;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.data.assign(grid.width * grid.height, -1);

    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const int index = grid.index(x, y);
        if (grid.occupied[index]) {
          occupancy_grid.data[index] = 100;
        } else if (grid.known_free[index]) {
          occupancy_grid.data[index] = 0;
        }
      }
    }

    nav_obstacle_grid_pub_->publish(occupancy_grid);
  }

  void publishFrontiers(
    const octomap_msgs::msg::Octomap & msg,
    const Grid2D & grid)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(deleteAllMarker(msg.header.frame_id, msg.header.stamp));

    visualization_msgs::msg::Marker marker;
    marker.header = msg.header;
    marker.ns = "frontier_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = grid.resolution;
    marker.scale.y = grid.resolution;
    marker.scale.z = 0.03;
    marker.color = makeColor(0.0F, 0.85F, 1.0F, static_cast<float>(frontier_alpha_));

    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        if (grid.frontier[grid.index(x, y)]) {
          marker.points.push_back(grid.cellCenter(x, y, floor_z_ + 0.03));
        }
      }
    }

    marker_array.markers.push_back(marker);
    frontier_marker_pub_->publish(marker_array);
  }

  void publishClusters(
    const octomap_msgs::msg::Octomap & msg,
    const Grid2D & grid,
    const std::vector<ScoredCandidate> & candidates)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(deleteAllMarker(msg.header.frame_id, msg.header.stamp));

    visualization_msgs::msg::Marker marker;
    marker.header = msg.header;
    marker.ns = "frontier_cluster_candidates";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const double sphere_size = std::max(grid.resolution * 3.0, 0.12);
    marker.scale.x = sphere_size;
    marker.scale.y = sphere_size;
    marker.scale.z = sphere_size;
    marker.color = makeColor(1.0F, 0.35F, 0.0F, 1.0F);

    for (const auto & candidate : candidates) {
      auto point = candidate.cluster.candidate;
      point.z = floor_z_ + sphere_size * 0.5;
      marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
    cluster_marker_pub_->publish(marker_array);
  }

  void publishCandidatePoses(
    const octomap_msgs::msg::Octomap & msg,
    const std::vector<ScoredCandidate> & candidates)
  {
    geometry_msgs::msg::PoseArray poses;
    poses.header = msg.header;
    poses.poses.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      poses.poses.push_back(makeGoalPose(candidate.cluster.candidate, 0.0).pose);
    }
    candidate_pose_pub_->publish(poses);
  }

  void publishBestGoal(
    const octomap_msgs::msg::Octomap & msg,
    const Grid2D & grid,
    const std::optional<ScoredCandidate> & best_goal)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(deleteAllMarker(msg.header.frame_id, msg.header.stamp));

    if (!best_goal) {
      best_goal_marker_pub_->publish(marker_array);
      return;
    }

    const auto goal = makeGoalPose(best_goal->cluster.candidate, 0.0);
    geometry_msgs::msg::PoseStamped stamped_goal = goal;
    stamped_goal.header = msg.header;
    best_goal_pub_->publish(stamped_goal);

    visualization_msgs::msg::Marker marker;
    marker.header = msg.header;
    marker.ns = "best_frontier_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = goal.pose;
    marker.pose.position.z = floor_z_ + 0.18;
    const double size = std::max(grid.resolution * 4.0, 0.20);
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color = makeColor(0.1F, 1.0F, 0.25F, 1.0F);
    marker_array.markers.push_back(marker);
    best_goal_marker_pub_->publish(marker_array);
  }

  void publishVisualSelectedBestGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    geometry_msgs::msg::PoseStamped stamped_goal = goal;
    if (stamped_goal.header.frame_id.empty()) {
      stamped_goal.header.frame_id = "odom";
    }
    if (stamped_goal.header.stamp.sec == 0 && stamped_goal.header.stamp.nanosec == 0) {
      stamped_goal.header.stamp = get_clock()->now();
    }
    best_goal_pub_->publish(stamped_goal);

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(
      deleteAllMarker(stamped_goal.header.frame_id, stamped_goal.header.stamp));

    visualization_msgs::msg::Marker marker;
    marker.header = stamped_goal.header;
    marker.ns = "best_frontier_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = stamped_goal.pose;
    marker.pose.position.z = floor_z_ + 0.18;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color = makeColor(0.1F, 1.0F, 0.25F, 1.0F);
    marker_array.markers.push_back(marker);
    best_goal_marker_pub_->publish(marker_array);
  }

  geometry_msgs::msg::PoseStamped makeGoalPose(
    const geometry_msgs::msg::Point & point,
    double yaw) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position = point;
    pose.pose.position.z = floor_z_;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    return pose;
  }

  void recordCompletedScanRotation()
  {
    const std::size_t previous_voxels =
      has_last_spin_voxel_count_ ? last_spin_voxel_count_ : 0;
    const std::size_t new_voxels =
      latest_voxel_count_ > previous_voxels ? latest_voxel_count_ - previous_voxels : 0;

    last_spin_voxel_count_ = latest_voxel_count_;
    has_last_spin_voxel_count_ = true;
    recent_spin_voxel_gains_.push_back(new_voxels);
    while (recent_spin_voxel_gains_.size() > 3) {
      recent_spin_voxel_gains_.pop_front();
    }

    RCLCPP_INFO(
      get_logger(), "Scan rotation observed %zu new OctoMap leaf voxels.", new_voxels);
  }

  void requestNextScanRotation()
  {
    if (exploration_complete_) {
      return;
    }
    if (!initial_spin_enabled_) {
      initial_spin_complete_ = true;
      publishScanReady(true);
      return;
    }

    publishScanReady(false);
    initial_spin_started_ = false;
    initial_spin_complete_ = false;
    initial_spin_stop_count_ = 0;
    initial_spin_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    initial_spin_complete_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    has_last_goal_ = false;
    has_visual_selected_goal_ = false;
    last_goal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    goal_arrival_stable_count_ = 0;
  }

  void finishExploration(const std::string & reason)
  {
    if (exploration_complete_) {
      return;
    }

    exploration_complete_ = true;
    goal_active_ = false;
    publishVelocity(0.0, 0.0);
    goal_arrival_stable_count_ = 0;

#ifdef ACTIVE_3D_SLAM_HAS_NAV2
    if (current_goal_handle_) {
      nav2_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
    }
#endif

    RCLCPP_INFO(get_logger(), "Exploration complete: %s", reason.c_str());
  }

  void handleGoalReached(bool cancel_nav2_goal)
  {
    goal_active_ = false;
    publishVelocity(0.0, 0.0);

#ifdef ACTIVE_3D_SLAM_HAS_NAV2
    if (cancel_nav2_goal && current_goal_handle_) {
      nav2_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
    }
#else
    (void)cancel_nav2_goal;
#endif

    if (stop_after_first_goal_) {
      finishExploration("First frontier goal reached.");
      return;
    }

    requestNextScanRotation();
  }

  void updateExplorationStopConditions(
    const std::optional<ScoredCandidate> & best_goal,
    std::size_t candidate_count)
  {
    if (exploration_complete_ || goal_active_ || !initial_spin_complete_) {
      return;
    }

    const auto now = get_clock()->now();
    if (exploration_start_time_.nanoseconds() == 0) {
      exploration_start_time_ = now;
    }
    if (max_exploration_time_sec_ > 0.0 &&
      exploration_start_time_.nanoseconds() > 0 &&
      (now - exploration_start_time_).seconds() >= max_exploration_time_sec_)
    {
      finishExploration("Reached maximum exploration time.");
      return;
    }

    if (max_goal_count_ > 0 && sent_goal_count_ >= max_goal_count_) {
      finishExploration("Reached maximum exploration goal count.");
      return;
    }

    if (candidate_count == 0 || !best_goal) {
      finishExploration("No reachable frontier candidates remain.");
      return;
    }

    if (best_goal->information_gain < best_unknown_gain_threshold_) {
      finishExploration("Best unknown gain is below threshold.");
      return;
    }

    const auto low_gain_limit =
      static_cast<std::size_t>(std::max(0, low_voxel_gain_threshold_));
    if (recent_spin_voxel_gains_.size() >= 3 &&
      std::all_of(
        recent_spin_voxel_gains_.begin(), recent_spin_voxel_gains_.end(),
        [low_gain_limit](std::size_t gain) {return gain <= low_gain_limit;}))
    {
      finishExploration("Recent scan rotations added too few voxels.");
    }
  }

  void maybeSendNav2Goal(
    const octomap_msgs::msg::Octomap & msg,
    const std::optional<ScoredCandidate> & best_goal)
  {
    if (!best_goal || !auto_send_nav2_goal_) {
      return;
    }
    if (use_visual_goal_selector_) {
      return;
    }

    const auto now = get_clock()->now();
    if (exploration_complete_) {
      return;
    }
    if (!initial_spin_complete_) {
      return;
    }
    if (initial_spin_enabled_ &&
      (now - initial_spin_complete_time_).seconds() < post_spin_settle_sec_)
    {
      return;
    }
    if (goal_active_) {
      return;
    }
    if (last_goal_time_.nanoseconds() > 0 &&
      (now - last_goal_time_).seconds() < goal_update_period_sec_)
    {
      return;
    }
    if (has_last_goal_ &&
      distance2D(best_goal->cluster.candidate, last_goal_.pose.position) < goal_replan_distance_)
    {
      return;
    }

    auto goal_pose = makeGoalPose(best_goal->cluster.candidate, 0.0);
    goal_pose.header = msg.header;
    sendNav2Goal(
      goal_pose, "frontier score", best_goal->score, best_goal->information_gain,
      best_goal->distance);
  }

  void sendNav2Goal(
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const std::string & source,
    double score,
    double information_gain,
    double distance)
  {
#ifdef ACTIVE_3D_SLAM_HAS_NAV2
    if (!controller_active_ || !bt_navigator_active_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Nav2 is not active yet; waiting before sending frontier goal "
        "(controller=%s, bt_navigator=%s).",
        controller_active_ ? "active" : "inactive",
        bt_navigator_active_ ? "active" : "inactive");
      return;
    }

    if (!nav2_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Nav2 action server '%s' is not ready; best goal is only being published.",
        nav2_action_name_.c_str());
      return;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
        goal_active_ = static_cast<bool>(goal_handle);
        current_goal_handle_ = goal_handle;
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "Nav2 rejected the frontier goal.");
          if (!exploration_complete_) {
            requestNextScanRotation();
          }
        }
      };
    options.result_callback =
      [this](const GoalHandleNavigateToPose::WrappedResult & result) {
        const bool was_goal_active = goal_active_;
        goal_active_ = false;
        current_goal_handle_.reset();
        RCLCPP_INFO(
          get_logger(), "Nav2 frontier goal finished with code %d",
          static_cast<int>(result.code));
        if (exploration_complete_ || !was_goal_active) {
          return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          handleGoalReached(false);
          return;
        }
        requestNextScanRotation();
      };

    nav2_client_->async_send_goal(goal_msg, options);
    publishScanReady(false);
    last_goal_ = goal_pose;
    has_last_goal_ = true;
    last_goal_time_ = get_clock()->now();
    goal_arrival_stable_count_ = 0;
    ++sent_goal_count_;
    RCLCPP_INFO(
      get_logger(),
      "Sent frontier goal %d to Nav2 from %s: x=%.2f y=%.2f score=%.2f gain=%.0f dist=%.2f",
      sent_goal_count_, source.c_str(), goal_pose.pose.position.x, goal_pose.pose.position.y,
      score, information_gain, distance);
#else
    (void)goal_pose;
    (void)source;
    (void)score;
    (void)information_gain;
    (void)distance;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Built without nav2_msgs; best goal is published but not sent to Nav2.");
#endif
  }

  visualization_msgs::msg::Marker deleteAllMarker(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  int worldToCellX(const Grid2D & grid, double x) const
  {
    return static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution));
  }

  int worldToCellY(const Grid2D & grid, double y) const
  {
    return static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution));
  }

  std::size_t countTrue(const std::vector<bool> & values) const
  {
    return static_cast<std::size_t>(std::count(values.begin(), values.end(), true));
  }

  std::size_t countLeafVoxels(const octomap::OcTree & tree) const
  {
    return static_cast<std::size_t>(std::distance(tree.begin_leafs(), tree.end_leafs()));
  }

  std::string octomap_topic_;
  std::string traversability_grid_topic_;
  std::string nav_obstacle_grid_topic_;
  std::string frontier_marker_topic_;
  std::string cluster_marker_topic_;
  std::string candidate_pose_topic_;
  std::string scan_ready_topic_;
  std::string best_goal_topic_;
  std::string best_goal_marker_topic_;
  std::string selected_goal_topic_;
  std::string cmd_vel_topic_;
  double floor_z_;
  double floor_clearance_;
  double robot_radius_;
  double robot_height_;
  double min_obstacle_z_;
  double max_free_z_;
  double grid_resolution_;
  double map_padding_;
  int min_cluster_size_;
  int max_candidate_count_;
  int max_grid_cells_;
  std::string robot_frame_;
  double information_gain_weight_;
  double distance_weight_;
  bool auto_send_nav2_goal_;
  bool use_visual_goal_selector_;
  std::string nav2_action_name_;
  double goal_update_period_sec_;
  double goal_replan_distance_;
  bool stop_after_first_goal_;
  double goal_arrival_tolerance_;
  bool use_distance_goal_monitor_;
  int goal_arrival_stable_count_required_;
  double min_goal_active_sec_;
  double best_unknown_gain_threshold_;
  int low_voxel_gain_threshold_;
  double max_exploration_time_sec_;
  int max_goal_count_;
  bool initial_spin_enabled_;
  double initial_spin_start_delay_;
  double initial_spin_angle_;
  double initial_spin_angular_speed_;
  double post_spin_settle_sec_;
  double frontier_alpha_;
  bool goal_active_{false};
  bool has_last_goal_{false};
  bool has_visual_selected_goal_{false};
  bool controller_active_{false};
  bool bt_navigator_active_{false};
  bool exploration_complete_{false};
  bool initial_spin_started_{false};
  bool initial_spin_complete_{false};
  int initial_spin_stop_count_{0};
  int sent_goal_count_{0};
  std::size_t latest_voxel_count_{0};
  std::size_t last_spin_voxel_count_{0};
  bool has_last_spin_voxel_count_{false};
  std::deque<std::size_t> recent_spin_voxel_gains_;
  int goal_arrival_stable_count_{0};
  rclcpp::Time exploration_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_spin_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_spin_complete_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_goal_;
  geometry_msgs::msg::PoseStamped visual_selected_goal_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selected_goal_sub_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr
    controller_transition_sub_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr
    bt_transition_sub_;
  rclcpp::TimerBase::SharedPtr initial_spin_timer_;
  rclcpp::TimerBase::SharedPtr goal_monitor_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr nav_obstacle_grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidate_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_ready_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr best_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr best_goal_marker_pub_;
#ifdef ACTIVE_3D_SLAM_HAS_NAV2
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
#endif
};

}  // namespace active_3d_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<active_3d_slam::FrontierExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
