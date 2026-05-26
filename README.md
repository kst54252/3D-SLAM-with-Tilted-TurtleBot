# 3D-SLAM-with-Tilted-TurtleBot
3D-SLAM-with-Tilted-TurtleBot

## Isaac Sim 실행 준비

Isaac Sim에서는 시뮬레이터가 로봇, 센서, `/clock`을 내보내고 이 저장소는 ROS 쪽 탐색 스택만 실행한다.

필요한 Isaac ROS 토픽:

- `/cmd_vel`: Isaac Sim 로봇이 구독하는 `geometry_msgs/msg/Twist`
- `/odom`: Isaac Sim이 발행하는 `nav_msgs/msg/Odometry`
- `/scan`: Isaac Sim 라이다의 `sensor_msgs/msg/LaserScan`
- `/camera/image_raw`: 후보 지점 평가용 RGB `sensor_msgs/msg/Image`
- `/tf`, `/tf_static`: 최소 `odom -> base_footprint`, `base_footprint -> base_scan`

기본 실행:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot3_description isaac_explore.launch.py
```

Isaac Sim의 토픽/프레임 이름이 다르면 launch argument로 맞춘다:

```bash
ros2 launch turtlebot3_description isaac_explore.launch.py \
  cmd_vel_topic:=/cmd_vel \
  odom_topic:=/odom \
  scan_topic:=/scan \
  image_topic:=/camera/image_raw \
  odom_frame:=odom \
  base_frame:=base_footprint \
  lidar_frame:=base_scan
```

Isaac Sim이 TF를 내보내지 않는 경우에만 아래 옵션을 켠다. Isaac Sim이 이미 같은 TF를 발행한다면 중복 TF가 생기므로 끄는 것이 맞다.

```bash
ros2 launch turtlebot3_description isaac_explore.launch.py \
  robot_state_publisher:=true \
  publish_odom_tf:=true
```

LLM 요청/응답 확인:

```bash
ros2 topic echo /llm_exchange_debug --qos-durability transient_local --full-length --field data
```

Ollama는 기본값으로 `http://localhost:11434/api/chat`, 모델은 `gemma4:26b`를 사용한다.
