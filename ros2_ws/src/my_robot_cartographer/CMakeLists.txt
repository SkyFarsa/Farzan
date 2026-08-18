cmake_minimum_required(VERSION 3.8)
project(my_robot_cartographer)

find_package(ament_cmake REQUIRED)
find_package(cartographer_ros REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
