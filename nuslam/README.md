# NUSLAM - Feature-based EKF implementation of SLAM

## Nodes
- `slam` - SLAM implementation, including odometry, the EKF, simulated landmarks.

## HOWTO
- `ros2 launch nuslam nuslam.launch.py robot:=nusim` - will launch the necessary nodes to get the SLAM up and running in simulation. It will launch rviz for visualization and a separate teleop window to control the robot.

# Screen capture

- ![image](https://github.com/ME495-Navigation/slam-project-maxipalay/assets/41023326/31f93221-8e56-41b3-9a30-9d4ff05eab7b)


# Simulation test

## Launching

- on a pc: `ros2 launch nuslam unknown_data_assoc.launch.py`

## Results

After running the robot on a closed loop for a few laps as shown in the video, the results show the following final positions:

- Position according to odometry (x,y,theta): -1.27, 0.58, 2.07
- Position according to simulation (ground truth)(x,y,theta): -1.34, 0.40, -1.94
- Position according to EKF estimate (x,y,theta): -1.32, 0.43, -1.97

Odometry error: 0.07, 0.18, 0.13
EKF error: 0.02, 0.03, 0.03

# Real world test

## Launching:

- on the turtlebot: `ros2 launch nuslam turtlebot_bringup.launch.py`
- on a pc: `ros2 launch nuslam pc_bringup.launch.py`

## Results

Driving the robot around the course and taking it back to its initial pose (eye-measuring using a mark on the floor).

Final SLAM estimation (x,y,theta): (-0.02, 0.01, -0.06) 
Final odometry estimation (x,y,theta): (0.02, 0.24, -0.67)

The SLAM outperforms the Odometry just by going a couple of rounds on the course.