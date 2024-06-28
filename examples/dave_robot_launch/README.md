## Examples

### 1. Launching REXROV in an empty world

To launch the rexrov model in an empty world, follow these steps:

1. Build and source the workspace:

   ```bash
   colcon build && source install/setup.bash
   ```

2. Launch the model using the specified launch file:

   ```bash
   ros2 launch dave_robot_launch robot_in_world.launch.py z:=2.0 namespace:=rexrov world_name:=empty.sdf paused:=false
   ```

### 2. Launching REXROV in dave_ocean_waves.world

To launch the rexrov model in an underwater world, follow these steps:

1. Build and source the workspace:

   ```bash
   colcon build && source install/setup.bash
   ```

2. Launch the model using the specified launch file:

   ```bash
   ros2 launch dave_robot_launch robot_in_world.launch.py z:=-5 namespace:=rexrov world_name:=dave_ocean_waves paused:=false
   ```
