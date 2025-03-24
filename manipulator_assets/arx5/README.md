# README: Integration of Ark Infinite ARX and MoveIt Algorithm

## 1. Project Overview
This project aims to integrate the Ark Infinite ARX platform's code with the MoveIt algorithm for more efficient and intelligent robot motion planning and control. ARX offers rich robot hardware interfaces and basic control functions, while MoveIt is a powerful motion - planning library in the robotics field.

## 2. Background
### Ark Infinite ARX
Ark Infinite ARX is a comprehensive robot development platform integrating various robot hardware drivers and control interfaces, providing a convenient development environment. https://www.arx-x.com/

### MoveIt Algorithm
MoveIt is an open - source robot motion planning framework with a series of algorithms and tools for complex motion planning tasks, featuring high flexibility and scalability. https://moveit.ai/

## 3. Project Goals
- Achieve seamless integration between ARX and MoveIt, ensuring smooth data interaction and collaboration.
- Leverage MoveIt's capabilities to provide smarter and more efficient motion planning for ARX - based robots.
- Develop example codes and documentation for easy adoption.

## 4. Project Structure
```plaintext
project_root/
├── arm_control/
│   ├── arx5.launch
│   ├── arx_arx5h.urdf
│   └── ...
├── arx5_config/
│   ├── config/
│   │   ├── controller.yaml
│   │   └── hw.yaml
│   │   └── servo.yaml
│   ├── moveit/
│   │   ├── config
│   │   └── launch
├── arx5_hw/
│   ├── include
│   ├── arx5_hw
│   └── ...
├── README.md
```

## 5. Installation and Configuration
### Requirements
- OS: Ubuntu 20.04 or later
- ROS: ROS Noetic or ROS 2 Foxy or later
- Ark Infinite ARX SDK: Installed and configured
- MoveIt: Installed following official documentation

### Steps
1. Clone the project:
```bash
git clone <repository URL>
cd <project root>
```
2. Compile the project:
```bash
catkin_make  # ROS 1
colcon build  # ROS 2
```
3. Configure the robot model: Place URDF and SRDF files in `moveit_config/robot_description` and adjust as needed.

## 6. Usage
1. Start the ARX arm and MOVEit! together
```bash
roslaunch arx5_config load_real.launch  # ROS 1
```
2. Start the simulatioin environment
```bash
roslaunch arx5_config load_gazebo.launch  # ROS 1
```
3. Run moveit bag
```bash
roslaunch arx5_config load_move_group.py  # ROS 1
```

## 7. Notes
- Ensure ARX hardware is properly connected and powered on.
- Adjust MoveIt planning parameters for complex tasks.
