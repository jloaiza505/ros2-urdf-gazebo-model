# ros2-urdf-gazebo-model

Physically consistent URDF robot model with modern Gazebo simulation and C++ control/testing nodes.

## Demo Goal

Build a small ROS 2 demo that proves:

1. URDF is physically valid (masses, inertia, limits, collisions).
2. Model behaves correctly in simulation.
3. Joint behavior is driven and validated from C++ nodes.

## C++-First Project Layout

```text
ros2_ws/
  src/
    robot_description_pkg/   # URDF/Xacro assets
    robot_sim_pkg/           # C++ ROS 2 nodes + launch files
```

## Prerequisites

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

## Build

```bash
cd ros2_ws
colcon build --packages-select robot_description_pkg robot_sim_pkg
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Launch Modes

1. URDF visualization only (no Gazebo):

```bash
ros2 launch robot_sim_pkg demo_rviz.launch.py
```

2. Modern Gazebo headless simulation + controllers:

```bash
ros2 launch robot_sim_pkg gazebo_demo.launch.py
```

3. Recommended portfolio mode: headless Gazebo + RViz2:

```bash
ros2 launch robot_sim_pkg gazebo_headless_rviz.launch.py
```

This runs:
1. `gz sim` server in headless mode
2. robot spawn from `robot_description`
3. `joint_state_broadcaster` + `forward_position_controller`
4. C++ `joint_command_node` (sinusoidal position commands)
5. RViz2 for visual monitoring

## Known Obstacles

On WSL2 with NVIDIA GPU, modern Gazebo GUI may crash due to renderer/driver constraints (OpenGL path limitations).  
Because of this, this project uses headless Gazebo (`-s`) plus RViz2 for visualization.

## Improvements

1. Re-enable native Gazebo GUI when graphics stack supports stable OpenGL 3.3+ rendering for `gz sim`.
2. Add CI smoke tests for controller startup (`joint_state_broadcaster`, `forward_position_controller`).
3. Add end-effector trajectory/metrics logging for repeatable portfolio benchmarks.
