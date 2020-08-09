# Vision-layer

## Overview

This package is used for the simulation of the autonomous stair climbing robot made for the Flipkart Grid Challenge 2.0.

**Keywords:** gazebo, world, plugin, sensor


This package has been tested under [ROS] Melodic and Ubuntu 18.04. 

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)







## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Gazebo] (Simulator for robotics) 

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/prakhar2001maheshwari/tripper.git
	cd ../
	catkin_make



## Usage

Launch the main world with

	roslaunch gazebo_simulation launchWorld.launch

Spawn the robot and payload with

	roslaunch gazebo_simulation spawnModel.launch
	roslaunch gazebo_simulation spawnPayload.launch

Launch the arm-controller with 

	rosrun gazebo_simulation arm_control


## Launch files

* **launchWorld.launch:** launches an empty world with staircases

* **spawnModel.launch:** spawns the robot model

* **spawnPayload.launch:** spwans the payload box  

* **spawnSDF.launch:** spawns the object specified by the SDF file







 

## Nodes

### Arm_Controller

Reads angle from the vision module and sets the servo to the specified angle


#### Subscribed Topics

* **`/climber/arm_state`** ([std_msgs/Float32])

	The angle at which the servo is currently set.



#### Published Topics

* **`/climber/arm_state`** ([std_msgs/Float32])

	The angle at which the servo is currently set.




[ROS]: http://www.ros.org
[Gazebo]: http://gazebosim.org
[rviz]: http://wiki.ros.org/rviz
[std_msgs/Float32]: http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html
