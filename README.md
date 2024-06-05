## Commands

### Spawn object

`ros2 run gazebo_ros spawn_entity.py -file "src/ros2_grasping/urdf/box.urdf" -entity "box" -x 0.5 -y -0.3 -z 0.75`

The parameter *name* is the name of the object declared in the urdf file. The file must be located in a folder named "urdf" in the specified package.

### Run program

`ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"`

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

### Run individual actions

`ros2 action send_goal -f /<action-type> <action-type-path> "<action-params>"`

For example, the command for the action "MoveJ" is:

`ros2 action send_goal -f /MoveJ ros2_data/action/MoveJ "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}, speed: 1.0}"`


## Action types

### MoveL

Moves the gripper part of the arm by a vector, so the whole arm moves to make the gripper cover the specified distance.

The components of the movement (x, y, z) are numbers from -1.0 to 1.0. They are relative to the maximum possible movement in that direction (front/back, left/right and up/down respectively), so, for example, if the arm is at the initial position, the allowed movement is 0.5 in every direction except down. This is because the joints of the arm would have to bend more than they can; some movements are in fact not allowed depending on the current position of the arm.