import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    pkg_simulation = get_package_share_directory('simulation')
    pkg_arm = get_package_share_directory('irb120_ros2_moveit2')

    launch_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_arm, 'launch', 'irb120_interface.launch.py'),
        )
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot",
        parameters=[
            {"robot_description": Command(["xacro ", os.path.join(pkg_simulation, "urdf/robot.urdf")])}
        ],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', '/robot/robot_description', '-x', '-0.0', '-y', '-0.5'],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="robot",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_simulation, "rviz/simulation.rviz")],
    )

    return LaunchDescription([
        launch_arm,
        robot_state_publisher,
        spawn_robot,
        # rviz
    ])