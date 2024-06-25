## Examples

### 1. Launching a Dave Model using Fuel URI

To launch a Dave model directly from a Fuel URI, follow these steps:

1. Build and source the workspace:

   ```bash
   colcon build && source install/setup.bash
   ```

2. Launch the model using the specified launch file:

   ```bash
   ros2 launch dave_demos model_in_empty_world.launch.py model_name:='mossy_cinder_block'
   ```

This method simplifies the process by pulling the model directly from Fuel, ensuring you always have the latest version without needing to manage local files.

### 2. Launching a Dave Model using Downloaded Model Files

If you prefer to use model files downloaded from Fuel, proceed as follows:

1. Add a hook within the `dave_object_models` package to configure the necessary environment variables for Gazebo model lookup.

   ```bash
   cd <path-to-dave_ws>/src/dave/models/dave_object_models
   mkdir hooks && cd hooks
   touch dave_object_models.dsv.in
   echo "prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;@CMAKE_INSTALL_PREFIX@/share/@PROJECT_NAME@" >> dave_object_models.dsv.in
   ```

2. Append the following line to the CMakeLists.txt file in the `dave_object_models` package:

   ```bash
   ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
   ```

3. Build and source the workspace:

   ```bash
   cd <path-to-dave_ws>
   colcon build && source install/setup.bash
   ```

4. Launch the model using the provided launch file:

   ```bash
   ros2 launch dave_demos model_in_empty_world.launch.py model_name:='nortek_dvl500_300'
   ```

This approach gives you more control over the models you use, allowing for offline use and customization. It's especially useful when working in environments with limited internet connectivity or when specific model versions are required.
