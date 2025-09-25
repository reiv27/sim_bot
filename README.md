# sim_bot

A ROS 2 package for launching a Gazebo simulation of a differential-drive robot.

## Dependencies

- ROS 2 Jazzy  
- Gazebo Harmonic  
- ros-jazzy-ros-gz-sim  
- ros-jazzy-xacro  
- ros-jazzy-ros-gz-bridge  
- ros-jazzy-ros-gz-image  

## Launch

```bash
ros2 launch sim_bot launch_sim.launch.py world:=path/to/world_file
