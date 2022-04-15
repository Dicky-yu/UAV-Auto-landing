# ROS udacity_bot robot model package
Author: Roberto Zegers R.

## Abstract
This package contains a URDF model of a differential drive robot equipped with a camera and laser sensor.

<img src="https://raw.githubusercontent.com/rfzeg/udacity_bot/master/docs/imgs/udacity_bot.png">
Fig.1 Image of the robot model in Gazebo (coke_can model shown for size comparison).  

## Default topics
+ Image Topic: /udacity_bot/camera1/image_raw
+ Image Info Topic:/udacity_bot/camera1/camera_info
+ Laser Scan Topic: /udacity_bot/laser/scan
+ Odometry Topic: /odom
+ Movement Commands: /cmd_vel

To spaw the robot into a running Gazebo simulation with a custom Odometry Topic append an argument like this:  
`$ roslaunch udacity_bot spawn_udacity_bot.launch odometryTopic:=odom_perfect`

## Repository architecture
### Directories
+ **urdf/** : (required) contains the files that generate the robot model and provide simulated actuators and sensors
+ **meshes/** : (required) contains the mesh files of the laser sensor
+ **config/** : (optional) contains YAML files that store the Navigation Stack configuration files for the robot
+ **rviz/** : (optional) contains Rviz configuration settings for displaying the robot model
+ **launch/** : (optional) contains launch files for starting the simulation / running nodes
+ **worlds/** : (optional) contains scene/environment files for Gazebo
+ **maps/** : (optional) contains the occupancy grid based maps required for navigation

### Robot model files
+ **udacity_bot.xacro** : the xacro file that generates the urdf description file of the robot
+ **udacity_bot.gazebo** : contains the Gazebo plugins that provide an interface to control the robot wheels and simulate the laser sensor

## Direct usage
- Clone this repository into a ROS catkin workspace
- Build and source the workspace
- To view this robot model on an empty Gazebo world: `$ roslaunch udacity_bot empty_world.launch`  
- To launch this package including the Jackal Race Gazebo world and Rviz: `$ roslaunch udacity_bot udacity_world.launch use_rviz:=true`  
or:  
- To spawn the robot into another already opened Gazebo world:  
`$ roslaunch udacity_bot spawn_udacity_bot.launch`  

If you want to move the robot using a keyboard you will also need to start a teleop node.  
To run the AMCL localization node and use the robot with the Navigation Stack type in a new window: `roslaunch udacity_bot amcl.launch`  

To view raw images on the topic /camera/rgb/image_raw, use:  
`$ rosrun image_view image_view image:=/udacity_bot/camera1/image_raw`  

## Mapping
First check that you satisfy all dependencies by running: `$ rospack find gmapping`  
You can use this robot to build a map since it includes the required odometry and laser sensor.
The inluded launch file `mapping.launch` will start Gazebo, load a predetermined world, and start the package **gmapping** properly configured, along with all other nodes required, including Rviz.  
To move the robot around a telop node is also required, for instance you can use:  
`$ rosrun rqt_robot_steering rqt_robot_steering`  
At the beginning there could be no map in Rviz, you may need to wait few second until it is generated.  

## Known Issues
+ Gazebo is crashing as it is starting up: Usually, it is enough to run it again (probably several times).
+ Most of the visual robot model shapes are not shown or are displayed collapsed on Rviz: make sure to run robot_state_publisher to publish transforms for all joints.  
  Do it by running `roslaunch udacity_bot robot_description.launch`  

This package has only been tested on Ubuntu 16.04 LTS with ROS Kinetic and Gazebo 7.0 and 7.15.
