#!/bin/bash

cd ~/robot-collaboration
. install/setup.bash

ros2 run gazebo_ros spawn_entity.py -file "src/ros2_grasping/urdf/box.urdf" -entity "box" -x 0.5 -y -0.3 -z 0.75

sleep 1

ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="example" -p ROBOT_MODEL:="irb120" -p EE_MODEL:="schunk"
