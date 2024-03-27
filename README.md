# EKF-based SLAM

https://github.com/maxipalay/ekf-slam/assets/41023326/2b1723bb-607d-425f-93e8-dac316ece29d

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

https://github.com/maxipalay/ekf-slam/assets/41023326/15553ec2-39d4-4105-bdd2-a6a1a4b19a7b

The odometry test recorded in the video showed a final error of roughly 0.29 meters. This test was performed using encoder odometry solely.

# Simulation test

Driving a circle around 4 landmarks in simulation. Green robot represents SLAM estimation, red robot represents ground truth in simulation, blue robot represents odometry-based localization.

https://github.com/maxipalay/ekf-slam/assets/41023326/a8fe9948-5daa-4884-9051-67ade433dcb1

# Real world test

Testing on the actual turtlebot! The SLAM outperforms the odometry with just a couple laps. First video in this readme shows this test.

Driving the robot around the course and taking it back to its initial pose (eye-measuring using a mark on the floor).

Final SLAM estimation (x,y,theta): (-0.02, 0.01, -0.06)
Final odometry estimation (x,y,theta): (0.02, 0.24, -0.67)
