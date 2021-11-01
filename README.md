# beginner_tutorials

## Project Description

The project is an introduction of Publishers and Subscrbers in ROS.
In this project, the publisher publishing the message "Hello-Terps" and the Subscriber subscribing it is demonstrated.

## Personnel

### Ameya Konkar 

UID:118191058

Master's Student at University of Maryland,College Park

## Overview

### Building the Program and Tests

```
Please install ROS using the link: http://wiki.ros.org/Installation/Ubuntu
After installation follow the following commands,
mkdir -p ~/catkin_ws
cd ~/catkin_ws
catkin init
mkdir src
cd src
sudo apt-get install git
git clone --recursive https://github.com/ameyakonk/beginner_tutorials.git
cd ..
catkin_make clean && catkin_make
source ./devel/setup.bash
Run the program talker.cpp: rosrun beginner_tutorials talker
(open other terminal)
Run the program listner.cpp: rosrun beginner_tutorials listner

```
