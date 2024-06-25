cmake_minimum_required(VERSION 3.8)
project(dave_robot_launch)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/dave_robot_launch
)

ament_package()
