#!/bin/bash

ros2 topic pub /action std_msgs/String "data: Moving away from table" -1
ros2 topic pub -t 1 /pippo/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
sleep 4

ros2 topic pub /action std_msgs/String "data: Stopping movement" -1
ros2 topic pub -t 1 /pippo/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
sleep 1

ros2 topic pub /action std_msgs/String "data: Moving back to table" -1
ros2 topic pub -t 1 /pippo/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
sleep 4

ros2 topic pub /action std_msgs/String "data: Stopping movement" -1
ros2 topic pub -t 1 /pippo/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
sleep 1

