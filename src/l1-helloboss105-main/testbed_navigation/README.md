# Testbed Navigation Package

This package provides navigation capabilities for the testbed robot using ROS2 Navigation2 (Nav2) stack.

## Package Structure

```
testbed_navigation/
├── config/
│   ├── amcl_params.yaml      # AMCL localization parameters
│   └── nav2_params.yaml      # Navigation2 stack parameters
├── launch/
│   ├── map_loader.launch.py      # Map server launch file
│   ├── localization.launch.py    # AMCL localization launch file
│   └── navigation.launch.py      # Complete navigation stack launch file
└── scripts/                      # Future scripts (if needed)
```

## Features

- **Map Loading**: Uses nav2_map_server to load the testbed world map
- **Localization**: AMCL (Adaptive Monte Carlo Localization) for robot pose estimation
- **Navigation**: Complete Nav2 stack with:
  - Global path planning using NavFn planner
  - Local path planning using DWB (Dynamic Window Approach)
  - Behavior trees for navigation logic
  - Collision monitoring and velocity smoothing
  - Recovery behaviors (spin, backup, wait)

## Usage

### Prerequisites

1. Make sure your workspace is built:
   ```bash
   cd ~/Pictures/project_ws
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ```

2. Start the Gazebo simulation with the testbed world:
   ```bash
   ros2 launch testbed_gazebo testbed_world.launch.py
   ```

### Testing Navigation Components

#### 1. Test Map Loading

Launch only the map server:
```bash
ros2 launch testbed_navigation map_loader.launch.py
```

Verify the map is loaded by checking topics:
```bash
ros2 topic list | grep map
ros2 topic echo /map --once
```

#### 2. Test Localization

Launch localization (includes map server):
```bash
ros2 launch testbed_navigation localization.launch.py
```

In RViz, you should be able to:
- See the map loaded
- Set initial pose estimate using "2D Pose Estimate" tool
- Observe the particle cloud from AMCL

#### 3. Test Complete Navigation

Launch the full navigation stack:
```bash
ros2 launch testbed_navigation navigation.launch.py
```

Testing navigation:
1. Open RViz: `rviz2`
2. Add displays for:
   - Map (/map)
   - Robot Model
   - LaserScan (/scan)
   - Path (/plan)
   - Local Costmap (/local_costmap/costmap)
   - Global Costmap (/global_costmap/costmap)
   - Particle Cloud (/particle_cloud)

3. Set initial pose using "2D Pose Estimate" tool
4. Send navigation goals using "2D Nav Goal" tool
5. Observe the robot planning and executing paths

### Launch File Parameters

#### map_loader.launch.py
- `map`: Path to map YAML file (default: testbed_bringup/maps/testbed_world.yaml)
- `use_sim_time`: Use simulation time (default: true)

#### localization.launch.py
- `map`: Path to map YAML file
- `use_sim_time`: Use simulation time (default: true)
- `autostart`: Auto-start lifecycle nodes (default: true)
- `params_file`: AMCL parameters file (default: config/amcl_params.yaml)
- `use_map_server`: Whether to start map server (default: true)

#### navigation.launch.py
- `namespace`: Robot namespace (default: '')
- `use_sim_time`: Use simulation time (default: true)
- `autostart`: Auto-start lifecycle nodes (default: true)
- `params_file`: Nav2 parameters file (default: config/nav2_params.yaml)
- `use_composition`: Use component composition (default: false)
- `use_respawn`: Respawn nodes on failure (default: false)
- `map`: Path to map YAML file
- `use_localization`: Whether to start localization (default: true)

## Configuration Files

### amcl_params.yaml
Contains AMCL-specific parameters including:
- Particle filter settings (min/max particles)
- Motion and sensor models
- Update thresholds
- Coordinate frames

### nav2_params.yaml
Comprehensive Nav2 configuration including:
- **bt_navigator**: Behavior tree configuration
- **controller_server**: DWB local planner settings
- **planner_server**: NavFn global planner settings
- **local_costmap**: Local costmap configuration
- **global_costmap**: Global costmap configuration
- **recoveries_server**: Recovery behavior settings
- **velocity_smoother**: Velocity smoothing parameters
- **collision_monitor**: Collision detection settings

## Troubleshooting

1. **Map not loading**: Check that testbed_bringup package is built and map files exist
2. **Localization not working**: Ensure laser scan topic (/scan) is publishing
3. **Navigation fails**: Check that all nav2 nodes are active using:
   ```bash
   ros2 lifecycle get /planner_server
   ros2 lifecycle get /controller_server
   ```
4. **Robot not moving**: Verify cmd_vel topic is being published and robot is receiving commands

## Topics and Services

Key topics:
- `/map`: Map data
- `/scan`: Laser scan data
- `/cmd_vel`: Velocity commands
- `/odom`: Odometry data
- `/amcl_pose`: Robot pose from AMCL
- `/plan`: Global path
- `/local_plan`: Local path

Key services:
- `/reinitialize_global_localization`: Reset AMCL particles
- `/navigate_to_pose`: Send navigation goal
- `/clear_entirely_global_costmap`: Clear global costmap
- `/clear_entirely_local_costmap`: Clear local costmap