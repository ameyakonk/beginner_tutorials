# beginner_tutorials

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Project Description

The project launch file to launch two nodes (talker and listner) at the same time. The program is equipped with appropriate 
warnings. It also provides a functionality to change output message using services. 

## Personnel

### Ameya Konkar 

UID:118191058

Master's Student at University of Maryland,College Park

## Overview

### Dependencies
This is a ROS package which needs [ROS Noetic](http://wiki.ros.org/Installation/Ubuntu) to be installed on Ubuntu 20.04. 

### Building the Program and Tests

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/ameyakonk/beginner_tutorials.git --branch Week10_HW
cd ..
catkin_make

```
### Run program using Rosrun

Open Terminal
```
roscore
```
Open other terminal

```
cd <path to catkin_ws>
source devel/setup.bash
rosrun beginner_tutorials talker

```
Open other terminal

```
cd <path to catkin_ws>
source devel/setup.bash
rosrun beginner_tutorials listner

```
### Run program with a launch file

Open Terminal
```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials listener_talker.launch

```
If you wish to change frequency of messages
Open Terminal
```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials listener_talker.launch frequency:=<desired frequency>

```
### Change the output message

Open Terminal
```
rosservice call /change_message "I am Ameya"

```
## Broadcasting TF Frame
The talker node broadcasts a TF frame /talk with varying translation and a constant rotation with parent frame as /world.

To check the result of the frame being braodcasted, first run the launch file in a terminal -
```
roslaunch beginner_tutorials listner_talker.launch frequency:=11
```
### Echo TF in terminal
In a new terminal, using the tf_echo tool, check if the /talk frame is actually getting broadcast to tf:
```
rosrun tf tf_echo /world /talk
```
### Visualize TF tree 
To view the tree of frames, use the rqt_tf_tree tool. Run the below command.
```
rosrun rqt_tf_tree rqt_tf_tree
```
### Save TF tree in PDF
The tree can be saved as a PDF using the view_frames tool. In a new terminal, navigate to the desired directory where the PDF is to be generated and run the following command:
```
rosrun tf view_frames
```
The tool listens to the TF broadcast for 5 seconds and saves the result in a pdf. To view the result run the following command:
```
evince frames.pdf
```
## Unit Testing with Rostest
To run the unit tests, execute the following in a new terminal
```
cd <path to catkin_ws>
catkin_make run_tests
```
## ROSBAG Recording
The published topics can be saved into a rosbag. The launch file has an argument to enable recording of the topics into a rosbag. To run the nodes and record the published topics, execute the folowing command in a terminal:
```
roslaunch beginner_tutorials listener_talker.launch rosbagRecord:=true
``` 
To view the rosbag, run the commands
```
cd <path to catkin_ws>/src/beginner_tutorials/results
rosbag play chatter.bag
```
