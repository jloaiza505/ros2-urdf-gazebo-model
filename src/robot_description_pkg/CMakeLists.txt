cmake_minimum_required(VERSION 3.8)
project(robot_description_pkg)

find_package(ament_cmake REQUIRED)

install(FILES
  LICENSE
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
