# Robot Localization

## Project Goal
Given a map of the world, use the robotics wheel odometry and LiDar sensors to localize itself in the world.

## Running the Particle Filter
To launch the particle filter, run the following:

**Launch rviz2 and pass configuration file**
``` 
$ rviz2 -d ~/ros2_ws/src/robot_localization/rviz/turtlebot_bag_files.rviz
```
**Launch the particle filter and load the map**
```
$ ros2 launch robot_localization test_pf.py map_yaml:=src/robot_localization/maps/mac_1st_floor_9_23.yaml
```
**Play the bag file**
```
$ ros2 bag play ros2_ws/src/robot_localization/bags/macfirst_floor_take_1 --clock
```


## Particle Filter: High-Level Concept Overview

### Algorithm Logic
The diagram below shows the workflow of a particle filter.

![alt text](image.png)



### 1. Initialize Particle Cloud
adf

### 2. Predict: Motion Update

### 3. Correct: Obsevation Update

### 4. Estimate: Robot Pose Update

### 5. Resample: Iterate