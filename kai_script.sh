#!/bin/bash

sudo apt-get install ros-kinetic-teb-local-planner-tutorials

source devel/setup.bash

roscd teb_local_planner_tutorials && roslaunch launch/robot_carlike_in_stage.launch

# 8=======D
