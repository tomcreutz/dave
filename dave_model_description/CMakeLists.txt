cmake_minimum_required(VERSION 3.8)
project(dave_model_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description launch meshes
  DESTINATION share/dave_model_description
)

ament_package()
