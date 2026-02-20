# Contributing

## Prerequisites

1. Ubuntu with ROS 2 Jazzy installed.
2. `colcon` and `rosdep` available in your shell.

## Setup

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build and Test

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src
source install/setup.bash
colcon test --base-paths src
colcon test-result --verbose
```

## Code Style

1. Keep package layout under `src/<package_name>/...`.
2. Update `package.xml` and `CMakeLists.txt` together when adding dependencies.
3. Keep launch/config/resource files installed through `CMakeLists.txt`.
