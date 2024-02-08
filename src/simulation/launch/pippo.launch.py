import os

from ament_index_python.packages import get_package_share_directory
from launch import Condition, LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    pkg_simulation = get_package_share_directory('simulation')
    pkg_arm = get_package_share_directory('irb1200_ros2_gazebo')

    launch_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_arm, 'launch', 'irb1200_simulation.launch.py'),
        )
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # name="pippo_state_publisher",
        namespace="pippo",
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {"robot_description": Command(["xacro ", os.path.join(pkg_simulation, "urdf/pippo.urdf")])}
        ],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'dolly', '-topic', '/pippo/robot_description', '-x', '5'],
        output='screen'
    )
    
    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state", "active",
            "-c", "pippo/controller_manager",
            "pippo_joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )
    
    load_diff_drive_controller = ExecuteProcess(
        name="activate_diff_drive_base_controller",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state", "active",
            "-c", "pippo/controller_manager",
            "pippo_diff_drive_controller",
        ],
        shell=False,
        output="screen",
    )

    return LaunchDescription([
        launch_arm,
        robot_state_publisher,
        spawn_entity,
        # robot_localization_node
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
    ])