#!/bin/bash

cd ~/robot-collaboration
. install/setup.bash

ros2 run gazebo_ros spawn_entity.py -file "src/ros2_grasping/urdf/box.urdf" -entity "box" -x 0.5 -y -0.3 -z 0.75

sleep 0.1

ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="example" -p ROBOT_MODEL:="irb120" -p EE_MODEL:="schunk"

sleep 0.2

echo "ATTACHING"
gz topic -p /polygon/join -m "data:'attach'"

sleep 0.2

echo "MOVING"
./move.sh

sleep 0.2

echo "DETACHING"
gz topic -p /polygon/join -m "data:'detach'"

sleep 0.2

ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="example_reverse" -p ROBOT_MODEL:="irb120" -p EE_MODEL:="schunk"
