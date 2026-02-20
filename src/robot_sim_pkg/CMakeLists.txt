cmake_minimum_required(VERSION 3.8)
project(robot_sim_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_sim_node src/robot_sim_node.cpp)
ament_target_dependencies(robot_sim_node rclcpp sensor_msgs)

add_executable(joint_command_node src/joint_command_node.cpp)
ament_target_dependencies(joint_command_node rclcpp std_msgs)

target_compile_features(robot_sim_node PUBLIC cxx_std_17)
target_compile_features(joint_command_node PUBLIC cxx_std_17)

install(TARGETS
  robot_sim_node
  joint_command_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  LICENSE
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
