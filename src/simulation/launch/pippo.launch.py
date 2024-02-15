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

    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory('simulation')
    pkg_arm = get_package_share_directory('irb1200_ros2_gazebo')
    # pkg_amazon = get_package_share_directory('custom_robots')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo, 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': os.path.join(pkg_arm, 'worlds', 'irb1200.world')}.items(),
    )

    launch_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_arm, 'launch', 'irb1200_simulation.launch.py'),
        )
    )

    # launch_amazon_robot = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_amazon, 'launch', 'spawn_amazon_model.launch.py')
    #     )
    # )

    # amazon_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[
    #         {"robot_description": Command(["xacro ", os.path.join(pkg_amazon, "urdf/amazon_robot.urdf")])}
    #     ],
    # )

    # amazon_joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher'
    # )

    # spawn_amazon_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'amazon_robot', '-topic', '/robot_description', '-y', '5'],
    #     output='screen'
    # )
    
    pippo_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="pippo",
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {"robot_description": Command(["xacro ", os.path.join(pkg_simulation, "urdf/pippo.urdf")])}
        ],
    )

    spawn_pippo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'dolly', '-topic', '/pippo/robot_description', '-x', '-5'],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="pippo",
        name="rviz2",
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="screen",
        arguments=["-d", os.path.join(pkg_simulation, "rviz/simulation.rviz")],
    )

    return LaunchDescription([
        # gazebo,
        launch_arm,
        # launch_amazon_robot,
        # amazon_state_publisher,
        # amazon_joint_state_publisher,
        # spawn_amazon_robot,
        pippo_state_publisher,
        spawn_pippo,
        rviz
    ])