cmake_minimum_required(VERSION 3.8)
project(active_3d_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs QUIET)

add_executable(frontier_extractor_node src/frontier_extractor_node.cpp)
target_compile_features(frontier_extractor_node PUBLIC cxx_std_17)
ament_target_dependencies(
  frontier_extractor_node
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  octomap_msgs
  rclcpp_action
  rclcpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
if(nav2_msgs_FOUND)
  ament_target_dependencies(frontier_extractor_node nav2_msgs)
  target_compile_definitions(frontier_extractor_node PRIVATE ACTIVE_3D_SLAM_HAS_NAV2)
endif()
target_link_libraries(frontier_extractor_node ${OCTOMAP_LIBRARIES})

install(TARGETS
  frontier_extractor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
