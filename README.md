# Robotics Systems Design Project

GitHub Repository for Team 8 of the MSc Robotics 2024/25 at the University of Manchester for module Robotic Systems Design Project.
This repository serves as the collaborative workspace for developing the robot for this project.

The goal of the project is to design a robot that is capable of fully autononmous navigation and object retrieval from within an environment.

## Principle functionalities:
- SLAM and autononmous navigation
- Object detection
- Object retrieval

## Required packages:
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- Nav2: https://github.com/ros-navigation/navigation2
- rplidar_ros: https://github.com/Slamtec/rplidar_ros
- robot_localization: https://github.com/cra-ros-pkg/robot_localization
- m-explore-ros2: https://github.com/robo-friends/m-explore-ros2
- interbotix_ws

## Installation Instructions:
This codebased has been designed to run on ROS2 Humble: https://docs.ros.org/en/humble/index.html

Clone this repository into your workspace:
```
git clone https://github.com/alyhatem/RoboticsSystemsDesignProject.git
```

Then clone dependency packages inside the /src folder of the cloned repository:
```
git clone https://github.com/robo-friends/m-explore-ros2.git
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
git clone https://github.com/cra-ros-pkg/robot_localization.git
```

Next install dependencies using APT:
```
apt-get install ros-humble-slam-toolbox
apt-get install ros-humble-navigation2
```

## Running the package
To run on an indentical robot (LeoRover, RPLiDAR A2M12, Intel RealSense D435, Trossen Robotics PX-100) simply launch with:
```
ros2 launch hardware_package hardware.launch.py
```
This launch file launches all functionality of the robot.
