# Mobile Robot Simulation and Autonomous Navigation

This repository contains a ROS 2 workspace for a differential-drive mobile robot developed in simulation. The project combines robot modeling, Gazebo simulation, sensor bridging, localization, SLAM, Nav2 autonomous navigation, frontier-based exploration, and camera-based object detection.

The goal was to build a complete mobile robotics pipeline: create the robot model, simulate its sensors and physics, localize it, build a map, navigate inside different worlds, and add higher-level autonomy strategies for exploration and target detection.

## Project Highlights

- Custom mobile robot described with URDF/Xacro
- Gazebo simulation with differential drive, lidar, RGB camera, depth camera, and IMU
- ROS 2 and Gazebo topic bridging with `ros_gz_bridge`
- State estimation with `robot_localization` EKF
- Mapping with `slam_toolbox`
- Autonomous navigation with Nav2
- MPPI controller configured for differential-drive motion
- Frontier exploration node that selects unknown map boundaries as goals
- OpenCV-based color and shape detection node using RGB and depth images
- Multiple Gazebo worlds and saved map files for testing

## Repository Structure

```text
ros2_ws/
|-- src/
|   |-- mobile_robot_description/
|   |   |-- urdf/mobile_robot.urdf.xacro
|   |   `-- launch/display.launch.py
|   `-- mobile_robot_gazebo/
|       |-- config/
|       |   |-- bridge.yaml
|       |   |-- ekf.yaml
|       |   |-- nav.yaml
|       |   `-- slam.yaml
|       |-- exploration/
|       |   |-- frontier_explorer.py
|       |   `-- detect_shape.py
|       |-- launch/
|       |   |-- basic.launch.py
|       |   |-- simulation.launch.py
|       |   |-- frontier_exploration.launch.py
|       |   `-- navigation_camera.launch.py
|       |-- maps/
|       |-- rviz/
|       `-- worlds/
|-- build/
|-- install/
`-- log/
```

## Tools and Technologies

| Tool | Usage in this project |
| --- | --- |
| ROS 2 | Main robotics middleware, launch system, nodes, topics, actions, and transforms |
| Gazebo Sim | Physics simulation, robot spawning, worlds, sensors, and differential-drive plugin |
| URDF/Xacro | Parametric robot model, links, joints, sensors, inertial properties, and Gazebo plugins |
| RViz2 | Robot visualization, TF tree, map, costmaps, laser scan, and navigation goals |
| Nav2 | Path planning, behavior tree navigation, costmaps, controller server, and recovery behaviors |
| `slam_toolbox` | Online mapping from lidar data |
| `robot_localization` | EKF fusion of odometry and IMU data |
| `ros_gz_bridge` | Bridge between Gazebo topics and ROS 2 topics |
| Python / `rclpy` | Custom autonomy nodes |
| OpenCV / `cv_bridge` | RGB image processing and color/shape detection |

## Robot Model

The robot is a compact differential-drive platform with:

- Two actuated side wheels
- One front caster wheel
- A 2D lidar for mapping and obstacle detection
- An RGB camera for visual perception
- A depth camera for estimating object position in 3D
- An IMU for localization

The model is defined in `src/mobile_robot_description/urdf/mobile_robot.urdf.xacro`. The Gazebo plugins publish wheel odometry and joint states, while the simulated sensors publish lidar, camera, depth, and IMU data.

## System Strategy

### 1. Simulation and Sensor Bridge

The robot is spawned in Gazebo from the Xacro description. Gazebo publishes simulated sensor data, and `ros_gz_bridge` converts the Gazebo messages into ROS 2 topics:

- `/scan`
- `/imu`
- `/odom`
- `/joint_states`
- `/camera/image_raw`
- `/camera/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`
- `/cmd_vel`

This makes the simulated robot behave like a real ROS 2 robot from the navigation stack point of view.

### 2. Localization

The EKF node from `robot_localization` fuses:

- Wheel odometry from Gazebo
- IMU angular velocity

The output is published as `/odometry/filtered`, which is used by Nav2. The EKF also publishes the `odom` to `base_footprint` transform.

### 3. Mapping

`slam_toolbox` runs in online mapping mode using the lidar scan topic `/scan`. It creates the `map` frame and publishes the occupancy grid used by exploration and navigation.

### 4. Navigation

Nav2 is configured in `src/mobile_robot_gazebo/config/nav.yaml`.

The navigation stack uses:

- `NavfnPlanner` with A* enabled for global planning
- MPPI controller for local trajectory control
- Local and global costmaps using lidar obstacle data
- Inflation layers to keep distance from obstacles
- Collision monitor for safer velocity commands

### 5. Frontier Exploration

The `frontier_explorer.py` node subscribes to `/map` and searches for frontier cells. A frontier is a free cell next to an unknown cell. The node randomly selects one frontier and sends it to Nav2 with the `NavigateToPose` action.

This strategy allows the robot to explore unknown areas by continuously moving toward the boundary between mapped and unmapped space.

### 6. Camera-Based Object Detection

The `detect_shape.py` node subscribes to RGB, depth, camera info, and costmap data. It uses OpenCV to:

- Convert RGB images to HSV
- Segment red, green, blue, and yellow objects
- Extract contours
- Estimate shape from contour geometry
- Select a yellow sphere as the target object
- Use the depth image and camera intrinsics to project the target into 3D
- Transform the target point into the `map` frame with TF2
- Send a Nav2 goal to the detected object

If no target is detected, the robot selects random free cells from the global costmap to continue moving and searching.

## Build

From the root of the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you use another ROS 2 distribution, replace `jazzy` with your installed version.

## Launch Commands

### Display the Robot Model

Use this to check the URDF/Xacro model in RViz:

```bash
ros2 launch mobile_robot_description display.launch.py
```

### Launch Gazebo, RViz, EKF, SLAM, and Nav2

This is the main simulation launch file:

```bash
ros2 launch mobile_robot_gazebo simulation.launch.py
```

By default it launches `simple_test.world`.

To launch another world:

```bash
ros2 launch mobile_robot_gazebo simulation.launch.py world:=maze_corridor.world
```

Available worlds:

```text
simple_test.world
empty.world
obstacle.world
maze_corridor.world
branching_path_markers.world
```

### Launch Basic Simulation Only

This launches Gazebo, the robot, the bridge, EKF, and RViz, without SLAM or Nav2:

```bash
ros2 launch mobile_robot_gazebo basic.launch.py
```

### Run Frontier Exploration

Start the main simulation first:

```bash
ros2 launch mobile_robot_gazebo simulation.launch.py
```

Then, in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_robot_gazebo frontier_exploration.launch.py
```

### Run Camera Detection and Target Navigation

Start the main simulation first, preferably in a world containing colored objects:

```bash
ros2 launch mobile_robot_gazebo simulation.launch.py world:=branching_path_markers.world
```

Then, in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_robot_gazebo navigation_camera.launch.py
```

## Test and Debug Commands

### Run Package Tests

Run all tests in the workspace:

```bash
colcon test
colcon test-result --verbose
```

Run tests for one package:

```bash
colcon test --packages-select mobile_robot_gazebo
colcon test-result --verbose
```

### Check Available ROS 2 Topics

```bash
ros2 topic list
```

### Inspect Sensor Data

```bash
ros2 topic echo /scan
ros2 topic echo /imu
ros2 topic echo /odometry/filtered
```

### Check Camera Streams

```bash
ros2 topic echo /camera/camera_info
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw
```

### Send a Manual Velocity Command

Use this only when the simulation is running:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

### Send a Nav2 Goal from the Terminal

Example goal in the `map` frame:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Visualize the TF Tree

```bash
ros2 run tf2_tools view_frames
```

This generates a PDF showing the current transform tree.

### Save a Map

After SLAM has built a map:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Main ROS Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command sent to Gazebo differential drive |
| `/odom` | `nav_msgs/msg/Odometry` | Raw simulated wheel odometry |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF odometry used by Nav2 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Lidar scan for SLAM and costmaps |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy grid produced by SLAM |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | RGB camera image |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth camera image |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 action used by custom autonomy nodes |
| `/detect_shape/debug_image` | `sensor_msgs/msg/Image` | Debug image with detected contours |
| `/detect_shape/goal_marker` | `visualization_msgs/msg/Marker` | RViz marker for selected goal |

## What I Implemented

- Designed the robot description with Xacro, including physical dimensions, inertias, joints, sensors, and Gazebo plugins
- Created Gazebo launch files and multiple test worlds
- Configured the ROS-Gazebo bridge for sensors, odometry, joint states, clock, and velocity commands
- Configured EKF localization from odometry and IMU
- Integrated SLAM Toolbox for online mapping
- Integrated and tuned Nav2 for autonomous navigation
- Implemented a frontier exploration node using occupancy-grid analysis
- Implemented a camera perception node using OpenCV, depth projection, TF2 transforms, and Nav2 goals
- Added RViz configuration and map files for development and testing

## Possible Improvements

- Replace random frontier selection with scoring based on distance, information gain, and obstacle clearance
- Add unit tests for frontier detection and image-processing utilities
- Add launch arguments for enabling or disabling SLAM, Nav2, RViz, and perception modules
- Improve target approach behavior by stopping at a safe offset from the detected object
- Add a recorded demo video or GIF to the README for GitHub presentation
