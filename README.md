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
