# ros2-urdf-gazebo-model Portfolio

## Title
ROS 2 Two-Link Arm Simulation with URDF Validation, Headless Gazebo, and C++ Joint Control

![Project hero: two-link arm in RViz with live joint motion](assets/images/hero_headless_gazebo_rviz_overview.png)

## Problem Statement
This project addresses a common robotics setup problem: getting a physically meaningful robot model running in simulation with reproducible control behavior.

The goals were:
- Build a URDF/Xacro model with valid inertial, collision, and joint limit data.
- Run the model in modern Gazebo (`gz sim`) with `ros2_control`.
- Drive joints from C++ nodes and observe behavior through ROS topics and RViz.

## System Architecture
The repository is organized as a ROS 2 workspace with two packages:
- `robot_description_pkg`: URDF/Xacro model files.
- `robot_sim_pkg`: C++ nodes, controller config, RViz config, and launch files.

Runtime data flow:
- `robot_state_publisher` publishes `robot_description` and TF.
- `ros_gz_sim` spawns the robot in headless Gazebo.
- `gz_ros2_control` exposes `/controller_manager`.
- `joint_command_node` publishes commands to `/forward_position_controller/commands`.
- `joint_state_broadcaster` publishes `/joint_states` for RViz monitoring.

![Architecture diagram: ROS 2 nodes, topics, and control path](assets/images/architecture_ros2_gz_control_flow.png)

## Technical Implementation
- Robot model: `ros2_ws/src/robot_description_pkg/urdf/two_link_arm.urdf.xacro`
- Controller config: `ros2_ws/src/robot_sim_pkg/config/controllers.yaml`
- Gazebo headless launch: `ros2_ws/src/robot_sim_pkg/launch/gazebo_demo.launch.py`
- Combined headless + RViz launch: `ros2_ws/src/robot_sim_pkg/launch/gazebo_headless_rviz.launch.py`
- C++ command node: `ros2_ws/src/robot_sim_pkg/src/joint_command_node.cpp`

The model uses two revolute joints (`joint1`, `joint2`), a fixed base (`world_to_base`), and a fixed end-effector frame (`ee_link`).

### Software Stack
- ROS 2 Humble
- C++ (`rclcpp`)
- `ros2_control`, `ros2_controllers`, `controller_manager`
- modern Gazebo integration: `ros_gz_sim` + `gz_ros2_control`
- RViz2 for visualization
- URDF/Xacro for robot description

### Core Engineering Decisions
- Use C++ nodes for command and state-publishing behavior to keep control logic explicit (`robot_sim_node`, `joint_command_node`).
- Run Gazebo in headless mode by default (`-s`) and visualize in RViz due GUI rendering issues on the current WSL2/NVIDIA stack.
- Split launch files by use-case:
  - RViz-only model check (`demo_rviz.launch.py`)
  - Headless simulation/control (`gazebo_demo.launch.py`)
  - Portfolio run (`gazebo_headless_rviz.launch.py`)
- Use `forward_command_controller` with position interface for a simple, readable control path.

## Key Challenges and Solutions
- Modern Gazebo GUI crashed on this environment (WSL2 + NVIDIA + renderer path).
  - Solution: keep simulation server headless and use RViz as the visual frontend.
- Controller startup originally blocked when plugin/config paths were inconsistent.
  - Solution: keep `gz_ros2_control` plugin and controller YAML wired through the installed package paths.
- Robot initially toppled during early setup.
  - Solution: anchor base with `world` link and fixed `world_to_base` joint.

## Results
- URDF parses successfully and link chain is valid (`world -> base_link -> link1 -> link2 -> ee_link`).
- In headless Gazebo mode, both controllers load and activate:
  - `joint_state_broadcaster`
  - `forward_position_controller`
- `joint_command_node` publishes sinusoidal commands at 50 Hz and robot joint states update accordingly.

![Evidence: active controllers in terminal](assets/images/result_active_controllers_terminal.png)
![Evidence: RViz showing moving links and TF frames](assets/images/result_rviz_motion_frames.png)
![Optional demo animation: headless Gazebo + RViz motion](assets/gifs/demo_headless_gazebo_rviz_motion.gif)

## Reproducibility
Install dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

Build and run portfolio mode:

```bash
cd ros2_ws
colcon build --packages-select robot_description_pkg robot_sim_pkg
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_sim_pkg gazebo_headless_rviz.launch.py
```

Validation commands:

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 topic echo /joint_states --once
ros2 topic echo /forward_position_controller/commands --once
check_urdf src/robot_description_pkg/urdf/two_link_arm.urdf
```

Key files for replication:
- `README.md`
- `ros2_ws/src/robot_description_pkg/urdf/two_link_arm.urdf.xacro`
- `ros2_ws/src/robot_sim_pkg/config/controllers.yaml`
- `ros2_ws/src/robot_sim_pkg/launch/gazebo_demo.launch.py`
- `ros2_ws/src/robot_sim_pkg/launch/gazebo_headless_rviz.launch.py`
- `ros2_ws/src/robot_sim_pkg/launch/demo_rviz.launch.py`
- `ros2_ws/src/robot_sim_pkg/src/joint_command_node.cpp`
- `ros2_ws/src/robot_sim_pkg/src/robot_sim_node.cpp`

![Reproducibility setup flow and commands](assets/images/repro_setup_and_launch_steps.png)

## What to Improve
- Add automated validation scripts that compare command vs state trajectories over a fixed run window.
- Add CI checks for launch and controller activation so regressions are caught early.
- Add rosbag-based metrics export (joint tracking error, limit compliance) for repeatable benchmark plots.
- Re-test GUI mode when moving to a native Linux environment or updated WSL graphics stack.
