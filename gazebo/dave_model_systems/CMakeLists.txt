cmake_minimum_required(VERSION 3.5)

# Define the project name
project(dave_model_systems)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-sim8 REQUIRED)

# Set version variables
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

message(STATUS "Compiling against Gazebo Harmonic")

# Add BasicSystem library
add_library(BasicSystem SHARED src/BasicSystem.cc)
add_library(UsblTransceiver SHARED src/UsblTransceiver.cc)
add_library(UsblTransponder SHARED src/UsblTransponder.cc)

# Include directories for BasicSystem
target_include_directories(BasicSystem PRIVATE include)
target_include_directories(UsblTransceiver PRIVATE include)
target_include_directories(UsblTransponder PRIVATE include)

# Link libraries for BasicSystem
target_link_libraries(BasicSystem
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

target_link_libraries(UsblTransceiver
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

target_link_libraries(UsblTransponder
gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Add FullSystem library
add_library(FullSystem SHARED src/FullSystem.cc)

# Include directories for FullSystem
target_include_directories(FullSystem PRIVATE include)

# Link libraries for FullSystem
target_link_libraries(FullSystem
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Specify dependencies for FullSystem using ament_target_dependencies
ament_target_dependencies(FullSystem
  rclcpp
  std_msgs
)

# Specify dependencies for FullSystem using ament_target_dependencies
ament_target_dependencies(UsblTransceiver
  rclcpp
  std_msgs
)

ament_target_dependencies(UsblTransponder
  rclcpp
  std_msgs
)


# Install targets
install(TARGETS BasicSystem FullSystem UsblTransceiver UsblTransponder
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Environment hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Configure ament
ament_package()
