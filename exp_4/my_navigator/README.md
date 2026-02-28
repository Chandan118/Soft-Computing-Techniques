# Hybrid Robot Navigation System

A ROS-based navigation system that combines global path planning with reactive obstacle avoidance using a hybrid approach. This package implements a custom navigation solution that can work alongside or replace the standard ROS navigation stack.

## Features

- Hybrid navigation combining global path planning with reactive obstacle avoidance
- Integration with RViz for goal setting and visualization
- Bug-algorithm inspired obstacle avoidance
- Adaptive waypoint sampling for smooth navigation
- Real-time path adjustment based on sensor data

## Prerequisites

- ROS (Robot Operating System)
- Python 2.7 or higher
- Required ROS packages:
  - geometry_msgs
  - nav_msgs
  - sensor_msgs
  - tf
  - actionlib_msgs

## Package Structure

```
my_navigator/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── check_system.launch
│   ├── navigate_direct.launch
│   ├── navigate.launch
│   ├── test_make_plan.launch
│   ├── test_muxer.launch
│   └── test_velocity.launch
├── param/
│   ├── costmap_common_params.yaml
│   ├── global_costmap_params.yaml
│   ├── local_costmap_params.yaml
│   └── move_base_params_global_only.yaml
└── scripts/
    ├── check_system.py
    ├── fuzzy_navigator.py
    ├── hybrid_navigator_direct.py
    ├── hybrid_navigator_edited.py
    ├── hybrid_navigator.py
    └── various test scripts
```

## Key Components

### Hybrid Navigator

The main navigation node (`hybrid_navigator_edited.py`) implements:
- Global path following using waypoints
- Reactive obstacle avoidance
- Dynamic path adjustment
- Integration with RViz for goal setting

### Parameters

Tunable parameters in the hybrid navigator:
- `GOAL_RADIUS`: 0.05 meters (distance to consider waypoint reached)
- `OBSTACLE_THRESHOLD`: 0.4 meters (minimum safe distance from obstacles)
- `LINEAR_SPEED`: 0.2 m/s (robot forward speed)
- `ANGULAR_SPEED`: 0.5 rad/s (robot turning speed)
- `WALL_FOLLOW_DISTANCE`: 0.3 meters (distance to maintain from walls during obstacle avoidance)

## Usage

1. Launch the navigation system:
```bash
roslaunch my_navigator navigate.launch
```

2. For direct navigation mode:
```bash
roslaunch my_navigator navigate_direct.launch
```

3. To test the system:
```bash
roslaunch my_navigator check_system.launch
```

## Setting Navigation Goals

1. Open RViz
2. Use the "2D Nav Goal" button to set a destination
3. The robot will plan and execute a path to the goal while avoiding obstacles

## Testing and Debug Tools

Several test scripts are provided:
- `test_make_plan.py`: Test path planning
- `test_path_sampling.py`: Test waypoint sampling
- `test_path_tracking.py`: Test robot path following
- `test_velocity_control.py`: Test velocity commands
- `test_muxer.py`: Test command multiplexing

## Costmap Configuration

Costmap parameters are configured in the `param/` directory:
- Global costmap settings
- Local costmap settings
- Common parameters
- Move base specific parameters

## License

This project is released under the MIT License. See the LICENSE file for details.

## Authors

- Chandan Sheikder
- Yichang Liu

## Acknowledgments

- ROS Community
- Navigation Stack Contributors
