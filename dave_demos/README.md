## Examples

### 1. Launching a Dave Model using Fuel URI

To launch a Dave model directly from a Fuel URI, follow these steps:

1. Build and source the workspace:
   ```bash
   colcon build && source install/setup.bash
   ```
2. Launch the model using the specified launch file:
   ```bash
   ros2 launch dave_demos mossy_cinder_block.launch.yaml
   ```

This method simplifies the process by pulling the model directly from Fuel, ensuring you always have the latest version without needing to manage local files.

### 2. Launching a Dave Model using Downloaded Model Files

If you prefer to use model files downloaded from Fuel, proceed as follows:

1. Set the resource path to your local model files:
   ```bash
   export GZ_SIM_RESOURCE_PATH=<Path-to-dave_ws>/src/dave/dave_model_description
   ```
2. Build and source the workspace:
   ```bash
   colcon build && source install/setup.bash
   ```
3. Launch the model using the provided launch file:
   ```bash
   ros2 launch dave_demos nortek_dvl500_300_bare_model.launch.yaml
   ```

This approach gives you more control over the models you use, allowing for offline use and customization. It's especially useful when working in environments with limited internet connectivity or when specific model versions are required.
