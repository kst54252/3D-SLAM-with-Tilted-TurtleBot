# 3D-SLAM-with-Tilted-TurtleBot
3D-SLAM-with-Tilted-TurtleBot

## 프로젝트 과정 설명

이 프로젝트는 30도 기울어진 2D 라이다를 장착한 TurtleBot3가 시뮬레이션 환경에서 주변을 3차원으로 인식하고, 미탐색 영역을 찾아 자율적으로 이동하는 과정을 구현한다. Isaac Sim 또는 Gazebo가 로봇의 물리 시뮬레이션과 센서 데이터를 제공하고, ROS 2 노드들이 센서 처리, 3D 맵 생성, frontier 추출, Nav2 기반 주행을 담당한다.

전체 동작 흐름

1. 제자리 360도 회전을 통해 현재 위치 주변의 3D point cloud를 생성한다.
2. 변환된 point cloud를 OctoMap에 입력하여 3D occupancy map을 만든다.
3. OctoMap에서 아직 탐색되지 않은 frontier 영역을 추출한다.
4. 후보 frontier들을 점수화 하여 상위 5개의 후보를 선택한다.
5. 후보 좌표들의 정보와 그 지점의 카메라 이미지로 로컬 vision LLM을 이용해 목표를 선택한다.
6. 선택된 goal을 Nav2로 전달하여 로봇을 이동시킨다.
7. 이동 후 위 과정을 반복한다.

주요 패키지 역할:

- `active_3d_description`: TurtleBot3 URDF, mesh, RViz 설정, 시뮬레이션 launch 파일을 관리한다.
- `active_3d_core`: scan 변환, odom TF 보조, cmd_vel 확인, 시각 기반 goal 선택 노드를 포함한다.
- `active_3d_slam`: OctoMap 기반 frontier 추출과 Nav2 탐색 launch/config를 포함한다.

## 사용 기술 스택

- ROS 2 Jazzy: 전체 노드 실행, 토픽 통신, launch 시스템, TF 관리
- Isaac Sim: TurtleBot3 물리 시뮬레이션, 라이다/카메라/odom/clock 데이터 제공
- Gazebo / gz-sim: Isaac Sim 대체용 로컬 시뮬레이션 환경
- TurtleBot3 URDF: 로봇 모델, 센서 위치, TF 구조 정의
- RViz2: 로봇 상태, TF, point cloud, OctoMap, frontier marker 시각화
- Nav2: 선택된 frontier goal까지의 경로 계획과 주행 제어
- OctoMap / octomap_server: 라이다 기반 3D occupancy map 생성
- PointCloud2: 기울어진 2D 라이다 scan을 3D point cloud로 변환하여 mapping에 사용
- Ollama: 로컬 vision LLM API 실행 환경
- `gemma4:26b`: 카메라 이미지를 기반으로 frontier 후보 goal을 선택하는 로컬 vision LLM 모델

기본 실행:
```bash
source install/setup.bash
ros2 launch turtlebot3_description tilted_lidar_sim.launch.py
```
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
