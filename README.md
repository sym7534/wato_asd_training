# WATonomous ASD Admissions Assignment
Created for WATonomous's ASD team.

## Project Outline
This project builds a simple autonomous navigation stack using ROS2:
- Costmap node: takes lidar data and publishes a local occupancy grid.
- Map memory node: fuses local costmaps with odometry into a global map.
- Planner node: runs A* pathfinding on the global map to generate a path.
- Control node: follows the path using a pure-pursuit controller.

## Planned Changes
- Fix the visual "jittering" of costmap in 3D render
- Optimize code
- Adapt simulation and code to work with moving obstacles
- Allow paths to be created with multiple goals

## Demo
Video: https://youtu.be/mIYiKe8uu4Q

## Usage
1. Clone repository to WSL ubuntu environment
2. cd ~/wato_asd_training, ./watod build, ./watod up
3. Connect to localhost:20000 in Foxglove app
4. Import a new Foxglove layout from config/wato_asd_training_foxglove_config .json
5. Use publishing feature to control robot.

