# paper_nav_experiment

A ROS package to demonstrate SLAM (gmapping) and autonomous navigation (move_base) for a simulated TurtleBot3 in a custom Gazebo world.  
This setup is intended as a baseline for experiments in robot navigation and mapping, as described in your research context.

## Features

- **Custom Gazebo World:** 10x10m indoor lab with obstacles ([worlds/lab_environment.world](worlds/lab_environment.world))
- **SLAM with GMapping:** Launch file for online mapping with TurtleBot3 and LiDAR
- **Autonomous Navigation:** Launch file for running move_base with AMCL localization on a saved map
- **RViz Visualization:** Pre-configured RViz setups for both SLAM and navigation
- **Tuned Parameters:** Costmaps and planner parameters for robust navigation in the provided world

## Project Structure

```
nav_experiment/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── run_slam.launch         # Launches Gazebo, robot, gmapping, and RViz for mapping
│   └── run_navigation.launch   # Launches Gazebo, robot, map_server, AMCL, move_base, and RViz for navigation
├── worlds/
│   └── lab_environment.world   # Custom SDF world file for Gazebo
├── param/
│   ├── base_local_planner_params.yaml
│   ├── costmap_common_params.yaml
│   ├── global_costmap_params.yaml
│   └── local_costmap_params.yaml
├── rviz/
│   ├── nav_config.rviz         # RViz config for navigation
│   └── slam_config.rviz        # RViz config for SLAM
└── maps/
    └── (your saved maps go here after SLAM)
```

## Getting Started

### 1. Build the Package

```sh
cd /path/to/your/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Run SLAM (Mapping)

This launches Gazebo, spawns the robot, starts gmapping, and opens RViz:

```sh
roslaunch paper_nav_experiment run_slam.launch
```

- Drive the robot around using teleop (e.g., `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`) to build a map.
- Save the map when done:

```sh
rosrun map_server map_saver -f ~/map_directory/lab_map
```

### 3. Run Navigation

After mapping, use the saved map for autonomous navigation:

```sh
roslaunch paper_nav_experiment run_navigation.launch map_file:=/full/path/to/lab_map.yaml
```

- Set navigation goals in RViz.

## Requirements

- ROS (tested with Kinetic/Melodic/Noetic)
- TurtleBot3 packages (`turtlebot3_gazebo`, `turtlebot3_navigation`, `turtlebot3_description`)
- Gazebo simulator
- GMapping (`ros-<distro>-gmapping`)
- RViz

## Parameter Tuning

- All costmap and planner parameters are in the [param/](param/) directory.
- The robot is assumed to be a TurtleBot3 Burger (default). Change the `model` argument in launch files if needed.

## File Descriptions

- **[launch/run_slam.launch](launch/run_slam.launch):** For mapping with SLAM.
- **[launch/run_navigation.launch](launch/run_navigation.launch):** For navigation using a saved map.
- **[worlds/lab_environment.world](worlds/lab_environment.world):** Custom world with obstacles.
- **[param/](param/):** Costmap and planner configuration.
- **[rviz/](rviz/):** RViz display configurations.

## Notes

- The `maps/` directory is empty by default. Save your maps here after SLAM.
- Make sure to set the TurtleBot3 model environment variable before launching:
  ```sh
  export TURTLEBOT3_MODEL=burger
  ```

## License

BSD

---

For questions or contributions, please edit the `package.xml` maintainer fields and submit issues or pull requests.