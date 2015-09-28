#!/bin/bash
source ./devel/setup.bash
catkin_make
. baxter.sh
rosrun baxter_tools enable_robot.py -e
