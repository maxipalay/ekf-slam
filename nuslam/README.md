# NUSLAM - Feature-based EKF implementation of SLAM

## Nodes
- `slam` - SLAM implementation, including odometry, the EKF, simulated landmarks.

## HOWTO
- `ros2 launch nuslam nuslam.launch.py robot:=nusim` - will launch the necessary nodes to get the SLAM up and running in simulation. It will launch rviz for visualization and a separate teleop window to control the robot.

# Screen capture

- ![image](https://github.com/ME495-Navigation/slam-project-maxipalay/assets/41023326/31f93221-8e56-41b3-9a30-9d4ff05eab7b)
