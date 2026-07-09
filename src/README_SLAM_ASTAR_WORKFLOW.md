# Warehouse UAV SLAM + ArUco + A* Workflow

이 문서는 `~/main_ws/src` 작업공간에서 진행한 PX4 SITL + Gazebo + ROS2 Humble 기반 warehouse mapping 프로젝트 인수인계용 정리이다.

목표는 UAV가 depth/RGB-D 카메라로 warehouse world를 SLAM하고, ArUco marker 위치를 저장한 뒤, 저장된 2D occupancy map에서 A*로 UGV 경로를 생성하는 것이다.

## 패키지 구조

주요 패키지:

- `warehouse_mapping`
  - SLAM bringup, RTAB-Map 설정, PX4 odometry bridge, offboard 경로, ArUco detector, map save/filter 노드
- `px4_depth_description`
  - PX4 x500 모델과 RGB-D camera 모델
- `ugv_path_planner`
  - 저장된 map + ArUco 위치 파일 기반 A* planner

주요 파일:

- `warehouse_mapping/launch/warehouse_slam_bringup.launch.py`
- `warehouse_mapping/launch/slam_mapping.launch.py`
- `warehouse_mapping/launch/offboard_path.launch.py`
- `warehouse_mapping/launch/aruco_goal_detector.launch.py`
- `warehouse_mapping/warehouse_mapping/offboard_path.py`
- `warehouse_mapping/warehouse_mapping/save_occupancy_map.py`
- `warehouse_mapping/warehouse_mapping/filter_saved_map.py`
- `warehouse_mapping/warehouse_mapping/pointcloud_timestamp_republisher.py`
- `ugv_path_planner/launch/aruco_astar_planning.launch.py`
- `ugv_path_planner/ugv_path_planner/astar_planner.py`
- `px4_depth_description/models/OakD-Lite-RGBD/model.sdf`

## 현재 방식 요약

최종적으로 쓰는 방식:

```text
Gazebo warehouse world
        ↓
PX4 SITL x500_depth_down
        ↓
RGB-D camera depth pointcloud
        ↓
RTAB-Map /rtabmap/grid_prob_map
        ↓
save_occupancy_map
        ↓
warehouse_map.yaml / warehouse_map.pgm
        ↓
filter_saved_map
        ↓
warehouse_map_filtered.yaml / warehouse_map_filtered.pgm
        ↓
A* planner
        ↓
/planned_path in RViz
```

현재 A*는 **실시간 map topic**이 아니라 **저장된 filtered map 파일**을 읽는다.

## 중요한 현재 설정

### Camera

Gazebo 렉을 줄이기 위해 RGB/Depth 모두 낮은 사양으로 설정했다.

```text
RGB:   320x240 @ 10Hz
Depth: 320x240 @ 10Hz
```

파일:

```text
px4_depth_description/models/OakD-Lite-RGBD/model.sdf
```

카메라 모델 변경 후에는 Gazebo/PX4를 완전히 종료 후 재실행해야 반영된다.

### Offboard path

가장 좋은 map이 나왔던 큰 세로 왕복 lawnmower 경로로 복구했다.

```text
altitude: 4.2m
max_speed: 0.04m/s
acceptance_radius: 0.70m
spawn: x=0.0, y=-5.5

x range: -11.0 ~ 11.0
y range: -10.5 ~ 10.5
lanes: [-11.0, -8.5, -6.0, -3.5, -1.0, 1.5, 4.0, 6.5, 9.0, 11.0]
```

복귀 구간에서는 `/mapping/active=false`가 publish된다.

### Mapping gate

포즈 오차로 복귀 중 같은 장애물이 두 번 찍히는 문제를 줄이기 위해 `/mapping/active` gate를 추가했다.

동작:

```text
offboard 대기/이륙 전: /mapping/active=false
스캔 구간:           /mapping/active=true
복귀/착륙 구간:      /mapping/active=false
```

`pointcloud_timestamp_republisher`는 `/mapping/active=false`일 때 `/depth_camera/points_synced`를 publish하지 않는다. 즉 RTAB으로 pointcloud가 들어가지 않는다.

확인:

```bash
ros2 topic echo /mapping/active
```

### RTAB-Map

현재 RTAB 설정은 `NormalsSegmentation=false`를 사용한다.

이유:

- 이전 방식은 박스 테두리만 잡히는 경우가 많았다.
- 현재 방식은 높이 기준으로 장애물을 판정하여 박스 내부까지 검정으로 채워진다.

핵심 설정:

```text
Grid/NormalsSegmentation false
Grid/MaxGroundHeight 0.12
Grid/MaxObstacleHeight 2.20
Grid/RangeMax 7.0
Grid/CellSize 0.05
GridGlobal/OccupancyThr 0.45
GridGlobal/ProbHit 0.78
GridGlobal/ProbMiss 0.48
```

`MaxGroundHeight=0.12`는 바닥 depth noise가 장애물로 잡히는 문제를 줄이기 위한 값이다.

## 실행 순서

각 터미널마다 아래 source를 먼저 한다.

```bash
cd ~/main_ws
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
source install/setup.bash
```

### 1. SLAM + Gazebo + bridge + RViz

```bash
cd ~/main_ws
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
source install/setup.bash

ros2 launch warehouse_mapping warehouse_slam_bringup.launch.py rviz:=true
```

RViz에서 live map은 다음 topic을 본다.

```text
/rtabmap/grid_prob_map
```

노란 선은 RTAB pose graph이다. 장애물이 아니다. 보기 싫으면 `MapGraph` display를 끈다.

### 2. PX4 SITL

별도 터미널:

```bash
cd ~/PX4-Autopilot

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=warehouse \
PX4_GZ_MODEL_NAME=x500_depth_down \
PX4_HOME_LAT=37.5665 \
PX4_HOME_LON=126.9780 \
PX4_HOME_ALT=0 \
make px4_sitl gz_x500
```

### 3. ArUco detector

```bash
cd ~/main_ws
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
source install/setup.bash

ros2 launch warehouse_mapping aruco_goal_detector.launch.py
```

검출된 marker 위치는 자동 저장된다.

```text
~/main_ws/maps/aruco_marker.yaml
```

확인:

```bash
cat ~/main_ws/maps/aruco_marker.yaml
```

`samples`가 충분히 쌓이면 안정적으로 검출된 것이다.

### 4. Offboard path

```bash
cd ~/main_ws
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
source install/setup.bash

ros2 launch warehouse_mapping offboard_path.launch.py
```

만약 드론 spawn이 Gazebo world 중앙 기준처럼 보이면 다음처럼 spawn 기준을 바꿔 테스트한다.

```bash
ros2 launch warehouse_mapping offboard_path.launch.py spawn_y:=0.0
```

## Map 저장, 필터, A*

SLAM이 충분히 완료되고 RViz에서 `/rtabmap/grid_prob_map`이 잘 보이면 아래 3단계를 수행한다.

### 1. Map 저장

```bash
cd ~/main_ws
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
source install/setup.bash

ros2 run warehouse_mapping save_occupancy_map \
  --ros-args \
  -p map_topic:=/rtabmap/grid_prob_map \
  -p output_prefix:=/home/dong/main_ws/maps/warehouse_map
```

결과:

```text
~/main_ws/maps/warehouse_map.yaml
~/main_ws/maps/warehouse_map.pgm
```

### 2. 저장된 map 필터

false positive와 pose drift로 생긴 얇은 선형 장애물을 제거한다.

추천 기본값:

```bash
ros2 run warehouse_mapping filter_saved_map \
  --input-yaml ~/main_ws/maps/warehouse_map.yaml \
  --output-prefix ~/main_ws/maps/warehouse_map_filtered \
  --unknown-mode free \
  --occupied-open-radius 0.08 \
  --occupied-close-radius 0.0 \
  --min-occupied-component-area 0.20
```

결과:

```text
~/main_ws/maps/warehouse_map_filtered.yaml
~/main_ws/maps/warehouse_map_filtered.pgm
```

파라미터 의미:

```text
--unknown-mode free
  회색 unknown 영역을 free로 처리

--occupied-open-radius 0.08
  얇은 검은 선형 노이즈 제거

--occupied-close-radius 0.0
  장애물끼리 이어붙이지 않음

--min-occupied-component-area 0.20
  작은 가짜 장애물 component 제거
```

만약 실제 작은 박스까지 사라지면 필터를 약하게 한다.

```bash
ros2 run warehouse_mapping filter_saved_map \
  --input-yaml ~/main_ws/maps/warehouse_map.yaml \
  --output-prefix ~/main_ws/maps/warehouse_map_filtered \
  --unknown-mode free \
  --occupied-open-radius 0.05 \
  --occupied-close-radius 0.0 \
  --min-occupied-component-area 0.15
```

### 3. Filtered map으로 A*

```bash
ros2 launch ugv_path_planner aruco_astar_planning.launch.py \
  map_yaml:=~/main_ws/maps/warehouse_map_filtered.yaml \
  obstacle_padding:=0.0 \
  clearance_radius:=0.20 \
  clearance_weight:=0.50 \
  goal_standoff_distance:=1.2
```

RViz marker 의미:

```text
green = start
blue  = 실제 ArUco marker 검출 위치
red   = A*가 접근할 goal 위치
```

RViz map topic:

```text
/planner/raw_map
```

Inflated map:

```text
/planner/inflated_map
```

`/planner/raw_map`은 필터된 실제 입력 map이고, `/planner/inflated_map`은 UGV radius/padding이 반영된 map이다.

## A* 주요 파라미터

현재 권장값:

```text
robot_radius: 0.20
obstacle_padding: 0.0 ~ 0.01
clearance_radius: 0.20 ~ 0.30
clearance_weight: 0.50 ~ 0.80
goal_standoff_distance: 1.2
unknown_is_occupied: false
```

장애물과 너무 멀리 떨어져 경로가 만들어지면:

```bash
obstacle_padding:=0.0 clearance_radius:=0.20 clearance_weight:=0.50
```

마커 근처 목표점이 벽에 너무 붙으면:

```bash
goal_standoff_distance:=1.2
```

더 멀리 떨어지고 싶으면:

```bash
goal_standoff_distance:=1.5
```

## 자주 생긴 문제와 해결

### Package not found

증상:

```text
Package 'warehouse_mapping' not found
Package 'ugv_path_planner' not found
```

해결:

```bash
cd ~/main_ws
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
source install/setup.bash
```

확인:

```bash
ros2 pkg list | grep warehouse_mapping
ros2 pkg list | grep ugv_path_planner
```

### 코드 수정 후 반영 안 됨

빌드 후 다시 source:

```bash
cd ~/main_ws
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
colcon build --packages-select warehouse_mapping ugv_path_planner px4_depth_description
source install/setup.bash
```

### Gazebo 렉

현재 camera는 `320x240 @ 10Hz`로 낮춰져 있다. 그래도 렉이 심하면 RViz에서 `MapCloud`, `PointCloud2`, `MapGraph` display를 꺼본다.

### RViz에서 map이 안 보임

확인:

```bash
ros2 topic list | grep -E "grid|map|planner"
ros2 topic hz /rtabmap/grid_prob_map
```

A* RViz에서 아무것도 안 보이면 A* 노드가 죽었거나 map path가 틀렸을 가능성이 있다.

실행 로그에서 다음 메시지를 본다.

```text
A* path found
Start is not free, snapped ...
Goal is not free, snapped ...
No path found ...
```

### 저장된 map이 RViz live map과 다르게 보임

RTAB live map은 확률 grid이고, 저장 map은 threshold/filter를 거친 PGM이다.

현재 저장 기본값:

```text
trinary=false
occupied_thresh=0.55
free_thresh=0.25
occupied_open_radius=0.10
occupied_close_radius=0.0
min_occupied_component_area=0.25
```

실제 작업에서는 저장 후 `filter_saved_map`을 한 번 더 적용한 `warehouse_map_filtered.yaml`로 A*를 돌리는 것을 권장한다.

### ArUco 위치가 이상함

기존 파일이 stale일 수 있다.

```bash
rm ~/main_ws/maps/aruco_marker.yaml
ros2 launch warehouse_mapping aruco_goal_detector.launch.py
```

다시 marker가 안정적으로 검출되면 `aruco_marker.yaml`이 새로 저장된다.

## 현재 최종 추천 명령어 3개

맵 저장:

```bash
ros2 run warehouse_mapping save_occupancy_map \
  --ros-args \
  -p map_topic:=/rtabmap/grid_prob_map \
  -p output_prefix:=/home/dong/main_ws/maps/warehouse_map
```

필터:

```bash
ros2 run warehouse_mapping filter_saved_map \
  --input-yaml ~/main_ws/maps/warehouse_map.yaml \
  --output-prefix ~/main_ws/maps/warehouse_map_filtered \
  --unknown-mode free \
  --occupied-open-radius 0.08 \
  --occupied-close-radius 0.0 \
  --min-occupied-component-area 0.20
```

A*:

```bash
ros2 launch ugv_path_planner aruco_astar_planning.launch.py \
  map_yaml:=~/main_ws/maps/warehouse_map_filtered.yaml \
  obstacle_padding:=0.0 \
  clearance_radius:=0.20 \
  clearance_weight:=0.50 \
  goal_standoff_distance:=1.2
```

