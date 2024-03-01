# ME495 Sensing, Navigation and Machine Learning For Robotics

* Maximiliano Palay
* Winter 2024

# Package List
This repository consists of several ROS packages

- `nuturtle_description` - Customized description for the turtlebot.
- `nusim` - Custom simulator with environment for turtlebot.
- `nusim_interfaces` - Custom service/message definitions for nusim.
- `nuturtle_control` - ROS2 control interface for a turtlebot running the custom MSR code.
- `nuturtle_control_interfaces` - Custom service/message definitions for `nuturtle_control`.
- `nuslam` - Feature based EKF SLAM implementation.

There is also a C++ library

- `turtlelib` - Provides classes and operations for control and sensing.
    - `diff_drive` - Operations necessary to calculate the kinematics of a differential drive robot.
    - `geometry2d` - General geometric calculation in 2 dimensions, including vectors, points, angles operations.
    - `se2d` - Definitions and operations for 2D rigid body motions.
    - `svg` - Accessory interface to visualize frames.

# Odometry test

https://github.com/ME495-Navigation/slam-project-maxipalay/assets/41023326/9d8e40f0-de09-4377-83a5-25810adaf671

The odometry test recorded in the video showed a final error of roughly 0.29 meters. This test was performed using encoder odometry solely.

# SLAM test

Driving a circle around 4 landmarks in simulation. Green robot represents SLAM estimation, red robot represents ground truth in simulation, blue robot represents odometry-based localization.

![image](https://github.com/ME495-Navigation/slam-project-maxipalay/assets/41023326/16e81a37-0733-46a7-81de-01b3f7a6e7ec)
