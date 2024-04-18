## Commands

### Spawn object

`ros2 run ros2_grasping spawn_object.py --package "ros2_grasping" --urdf "---" --name "---" --x 0.0 --y -0.0 --z 0.0`

### Run program

`ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"'`

Program file must be located in __ros2_execution/programs__

The robot model name is "*irb120*" and the ee (end effector) model is "*schunk*".


* For MoveJ ---> {'action': 'MoveJ', 'value': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}, 'speed': 1.0}
* For MoveL ---> {'action': 'MoveL', 'value': {'movex': 0.0, 'movey': 0.0, 'movez': 0.0}, 'speed': 1.0}
* For MoveR ---> {'action': 'MoveR', 'value': {'joint': '---', 'value': 0.0}, 'speed': 1.0}
* For MoveXYZW ---> {'action': 'MoveXYZW', 'value': {'positionx': 0.0, 'positiony': 0.0, 'positionz': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveXYZ ---> {'action': 'MoveXYZ', 'value': {'positionx': 0.0, 'positiony': 0.0, 'positionz': 0.0}, 'speed': 1.0}
* For MoveYPR ---> {'action': 'MoveYPR', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveROT ---> {'action': 'MoveROT', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveRP ---> {'action': 'MoveRP', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}, 'speed': 1.0}
* For MoveG:
    * To Open Gripper ---> {'action': 'GripperOpen'}
    * To Close Gripper ---> {'action': 'GripperClose'}
* For the ros2_grasping feature:
    * To attach object to end-effector ---> {'action': 'Attach', 'value': {'object': '---', 'endeffector': '---'}}
    * To detach object ---> {'action': 'Detach', 'value': {'object': '---'}}
