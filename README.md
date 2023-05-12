# Hierarchical-Task-Planning-with-Scene-Understanding-for-the-Automation-Home-Service-Robot

## Overview
Still Progressing
## Installation
This repository uses a list of others repository, for more detail you can check the following links

https://github.com/leggedrobotics/darknet_ros

https://github.com/dananau/GTPyhop

https://github.com/kivy/kivy


The package is only tested in the ROS Noetic 20.04, Gazebo 11, opencv 4.2.0.
The following instructions assume that you are already have the appropriate version of all software mentioned above (ROS, Gazebo, ...), and also install Rviz, move base, and Octomap.

```
# Clone the repository
mkdir -P ~/YOUR_WS_NAME/src
cd ~/YOUR_WS_NAME

# To maximize the performance of object detection 
catkin_make -DCMAKE_BUILD_TYPE=Release darknet_ros

# Otherwise 
catkin_make
```
## Simulation in Gazebo
To run the environment, you need to make sure that the model has beed added to the Gazebo model path.

```
# Open a terminal and type
export GAZEBO_MODEL_PATH=~/{WS_NAME}/src/mm3/models:${GAZEBO_MODEL_PATH}

# Or you can just add this line to your .bashrc file
vim ~/.bashrc 
# Add the above line (export .......) to your file.

cd ~/YOUR_WS_NAME
source devel/setup.bash
roslaunch mm3 gazebo_furnitured.launch
```
The Gazebo environment is downloaded from the AWS Robotics
After successfully launch the world, you should see a dual-arm mobile manipulator (DAMM) with a camera and lidar in a house.
![Screenshot from 2022-07-08 10-32-14](https://user-images.githubusercontent.com/55338365/226248982-a71d0ead-fc92-453d-be8d-2e54a2d39659.png)

### Basic control of DAMM
A list of python scripts is used for remote control the DAMM
```
# To control the mobile base, you need to open a new terminal 
cd ~/YOUR_WS_NAME
source devel/setup.bash
rosrun mm3 teleop_key.py

# You can control the robot with w/increase velocity, a/increase angular velocity(turn left), s/stop, d/decrease angular velocity(turn right), x/decrease velocity

# To Control the Arm
cd ~/YOUR_WS_NAME
source devel/setup.bash
rosrun mm3 arm_control.py

# Note that arm is control in the cartesian space, the way of controlling the arm is similar with control the mobile base.
```

## Navigation and octomapping
```
# To run the navigation stack
cd ~/YOUR_WS_NAME
source devel/setup.bash
roslaunch mm3 move_base.launch
## It will open a Rviz automatically, you can send goal in Rviz.

# Since we have already build the octomap, you can use the octomap /projected_map topic for navigation instead of using /map.
cd /home/po-lun/HSR/src/Hierarchical-Task-Planning-with-Scene-Understanding-for-the-Automation-Home-Service-Robot/mm3/path_planning/config
vim costmap_common_params.yaml
# change map topic from map to projected_map
map_topic: /projected_map

# Loading the octomap
cd ~/YOUR_WS_NAME
source devel/setup.bash
roslaunch octomap_server octomap_to_2Dmap.launch
```

![Screenshot from 2023-03-18 16-35-56](https://user-images.githubusercontent.com/55338365/226251843-eac3c604-959c-480b-bae8-854b523a85eb.png)

## Scene understanding


## GUI
```
# To run the navigation stack
cd ~/YOUR_WS_NAME
source devel/setup.bash
cd /src/Hierarchical-Task-Planning-with-Scene-Understanding-for-the-Automation-Home-Service-Robot/gui/nodes
python3 gui.py
```
![image](https://user-images.githubusercontent.com/55338365/226250911-d177c5e8-c583-447e-a237-ddf19ece675e.png)

you will see a very basic user interface like this, and you can enter the task which need to in the predefined list (open/close "ITEM", clean "LOC", pour "DRINK", pickup/put "ITEM") you want to do and press submit. 

## HTN planner
A HTN planner is making the decision for the robot
```
cd ~/YOUR_WS_NAME
source devel/setup.bash
rosrun htn_planner state_manager.py

# This node subscribe to gui input, it will automatically generate a list of action after you input the appropriate task in the gui.
```
For example, input: clean table

### plan [('move', 'trash_can'), ('open', 'trash_can'), ('move', 'table'), ('pickup', 'cola', 'table'), ('move', 'trash_can'), ('put', 'cola', 'trash_can'), ('move', 'table'), ('pickup', 'dish', 'table'), ('move', 'drain'), ('put', 'dish', 'drain')]

Note that in this example, we know that the cola and the dish is on the table, and the trash can is closed. But these state is supposed to obtain from our scene understanding algorithm, we will discuss about how we get the state in the next section.


## Object detection
```
cd ~/YOUR_WS_NAME
source devel/setup.bash
roslaunch darknet_ros yolo.launch
```
![Screenshot from 2023-03-18 22-33-12](https://user-images.githubusercontent.com/55338365/226253217-dc320ef3-b1c8-473e-946d-ac816891ddc7.png)


## Gripping
1. alignning the depth image and the rgb image
2. Use octomap to generate a collision free path
3. Use Jacobian to control our robot

## Demo
```
```
### Video
