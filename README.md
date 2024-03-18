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

![image](https://github.com/ME495-Navigation/slam-project-maxipalay/assets/41023326/b29dbed4-4ae7-4b05-8a15-ad73554f5f96)

# Real world test

Testing on the actual turtlebot! The SLAM outperforms the odometry with just a couple laps.

https://github.com/ME495-Navigation/slam-project-maxipalay/assets/41023326/28ba2547-83ba-43ec-9bb8-e3cf541f95de

![Screenshot from 2024-03-17 19-38-13](https://github.com/ME495-Navigation/slam-project-maxipalay/assets/41023326/506a3721-cbf0-42d2-bc8a-5014ece5e995)


Driving the robot around the course and taking it back to its initial pose (eye-measuring using a mark on the floor).

Final SLAM estimation (x,y,theta): (-0.02, 0.01, -0.06)
Final odometry estimation (x,y,theta): (0.02, 0.24, -0.67)
