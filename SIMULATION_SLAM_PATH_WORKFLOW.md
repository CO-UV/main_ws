# UAV Depth Mapping, SLAM, and Path Planning Workflow

## Goal

This document summarizes the full simulation flow we have been using:

1. Spawn a drone with a downward-facing depth camera in the warehouse world.
2. Fly the drone across the full warehouse using PX4 offboard control.
3. Run SLAM from the depth point cloud and PX4 odometry.
4. Save or reuse the generated occupancy map.
5. Run path planning on the saved map to generate an optimal route.

## High-Level Flow

```text
Gazebo world
  -> spawn depth-drone
  -> PX4 SITL + MicroXRCEAgent
  -> ros_gz_bridge depth topics
  -> robot_state_publisher camera TF
  -> px4_odometry_bridge
  -> RTAB-Map SLAM
  -> occupancy map
  -> A* path planner
```

## Main Nodes and Processes

### Simulation and Vehicle

- `gz sim`
  - Runs the warehouse simulation world.
- `PX4 SITL`
  - Publishes PX4 topics such as `/fmu/out/vehicle_local_position_v1`.
- `MicroXRCEAgent`
  - Bridges PX4 DDS traffic into ROS 2.
- `px4_depth_description/spawn_px4_depth_drone.launch.py`
  - Spawns the drone model with the downward depth camera.

### Sensor and TF

- `ros_gz_bridge/parameter_bridge`
  - Bridges:
    - `/clock`
    - `/depth_camera`
    - `/depth_camera/points`
    - `/camera_info`
- `robot_state_publisher`
  - Publishes the TF tree for the downward camera model.

### Flight Control

- `warehouse_mapping/offboard_path`
  - Sends offboard setpoints so the drone can sweep the whole warehouse.

### SLAM

- `warehouse_mapping/px4_odometry_bridge`
  - Converts PX4 local position into `/px4/odom`.
- `rtabmap`
  - Uses `/depth_camera/points` and `/px4/odom` to build the map.

### Path Planning

- `ugv_path_planner/astar_planner`
  - Loads a saved occupancy map and computes a path from start to goal.

## Relevant Launch Files

- [gazebo_warehouse.launch.py](/home/hong/HONG/src/warehouse_mapping/launch/gazebo_warehouse.launch.py:1)
- [spawn_px4_depth_drone.launch.py](/home/hong/HONG/src/px4_depth_description/launch/spawn_px4_depth_drone.launch.py:1)
- [camera_tf.launch.py](/home/hong/HONG/src/px4_depth_description/launch/camera_tf.launch.py:1)
- [slam_mapping.launch.py](/home/hong/HONG/src/warehouse_mapping/launch/slam_mapping.launch.py:1)
- [offboard_path.launch.py](/home/hong/HONG/src/warehouse_mapping/launch/offboard_path.launch.py:1)
- [astar_planner.launch.py](/home/hong/HONG/src/ugv_path_planner/launch/astar_planner.launch.py:1)

## Prerequisites

- ROS 2 Humble
- Gazebo Sim
- PX4 SITL environment
- `MicroXRCEAgent`
- Workspace built with `colcon`

Build once before running:

```bash
cd /home/hong/HONG
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Recommended Execution Order

Open separate terminals and source ROS 2 plus the workspace in each one:

```bash
source /opt/ros/humble/setup.bash
cd /home/hong/HONG
source install/setup.bash
```

### Terminal 1: Micro XRCE Agent

```bash
/home/hong/micro_xrce_agent.sh
```

Expected role:

- Makes PX4 ROS 2 topics available under `/fmu/...`

### Terminal 2: Gazebo Warehouse World

```bash
ros2 launch warehouse_mapping gazebo_warehouse.launch.py
```

Expected role:

- Starts the warehouse simulation world.

### Terminal 3: Spawn the Depth Drone

```bash
ros2 launch px4_depth_description spawn_px4_depth_drone.launch.py
```

Expected role:

- Spawns the drone with a downward-facing depth camera.

### Terminal 4: PX4 SITL

Run your usual PX4 SITL command that connects to Gazebo and publishes `/fmu` topics.

This step is required because:

- `offboard_path` depends on PX4 topics.
- `px4_odometry_bridge` depends on `/fmu/out/vehicle_local_position_v1`.

Check that PX4 is alive with:

```bash
ros2 topic list | grep /fmu
```

At minimum, these topics should exist:

- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_status`

### Terminal 5: SLAM Pipeline

```bash
ros2 launch warehouse_mapping slam_mapping.launch.py
```

This launch starts:

- `ros_gz_bridge/parameter_bridge`
- `robot_state_publisher`
- `warehouse_mapping/px4_odometry_bridge`
- `rtabmap`

Expected inputs:

- `/depth_camera/points`
- `/fmu/out/vehicle_local_position_v1`

Expected outputs:

- `/px4/odom`
- RTAB-Map map data

### Terminal 6: Offboard Sweep Flight

```bash
ros2 launch warehouse_mapping offboard_path.launch.py
```

Useful parameters:

```bash
ros2 launch warehouse_mapping offboard_path.launch.py \
  altitude:=3.2 \
  auto_arm:=true \
  auto_land:=true \
  acceptance_radius:=0.80 \
  max_speed:=0.15
```

Expected role:

- Moves the drone through the predefined warehouse sweep path.
- Provides the coverage needed for full-map SLAM.

### Terminal 7: Path Planning on the Saved Map

After the occupancy map is ready, run:

```bash
ros2 launch ugv_path_planner astar_planner.launch.py \
  map_yaml:=/home/hong/HONG/maps/warehouse_occupancy.yaml \
  start_x:=0.0 \
  start_y:=-10.0 \
  goal_x:=9.0 \
  goal_y:=9.0
```

Expected role:

- Loads the saved occupancy map.
- Computes an A* path from start to goal.

## Data Products

### Map Files

Current saved occupancy map files:

- [warehouse_occupancy.yaml](/home/hong/HONG/maps/warehouse_occupancy.yaml:1)
- [warehouse_occupancy.pgm](/home/hong/HONG/maps/warehouse_occupancy.pgm:1)

### Planner Input

The planner currently defaults to:

- `/home/hong/HONG/maps/warehouse_occupancy.yaml`

## What Each Stage Achieves

### Stage 1: Sensorized Drone Simulation

- The drone exists in Gazebo.
- The depth camera points downward.
- Camera TF is available.

### Stage 2: Warehouse Sweep

- PX4 offboard control sends the vehicle across the warehouse.
- The drone covers the full operating area instead of hovering in one place.

### Stage 3: SLAM

- Depth point clouds are fused with PX4 odometry.
- RTAB-Map reconstructs the environment and supports map generation.

### Stage 4: Path Planning

- The saved occupancy map is used as planner input.
- A* generates a feasible path with obstacle padding and clearance cost.

## Quick Health Checks

### Check Gazebo Depth Topics

```bash
ros2 topic list | grep depth_camera
```

### Check PX4 Topics

```bash
ros2 topic list | grep /fmu
```

### Check Converted Odometry

```bash
ros2 topic echo --once /px4/odom
```

### Check RTAB-Map Input Rate

```bash
ros2 topic hz /depth_camera/points
```

## Common Failure Points

- No `/fmu` topics:
  - PX4 SITL or `MicroXRCEAgent` is not running correctly.
- No `/depth_camera/points`:
  - Gazebo bridge is not running or the depth camera model was not spawned.
- `offboard_path` does not move the vehicle:
  - PX4 is not in offboard mode, not armed, or not publishing status/local position.
- SLAM does not grow:
  - Point cloud is missing, TF is broken, or odometry is not reaching RTAB-Map.
- Planner fails:
  - The occupancy map path is wrong or the start/goal is inside an occupied area.

## Summary

For the full workflow, the essential stack is:

- `gz sim`
- `PX4 SITL`
- `MicroXRCEAgent`
- `spawn_px4_depth_drone`
- `parameter_bridge`
- `robot_state_publisher`
- `px4_odometry_bridge`
- `offboard_path`
- `rtabmap`
- `astar_planner`

This is the complete chain from downward depth sensing to SLAM map creation and final path generation.
