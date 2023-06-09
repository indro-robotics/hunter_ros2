#!/usr/bin/env python3

import os
import sys
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    install_dir = get_package_prefix('hunter_se_description')

    # Installing Gazebo Model and Plugin Paths
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'
    pkg_tracer_description = get_package_share_directory(
        'hunter_se_description')

    ld = LaunchDescription()

    # Acquiring robot description XACRO file
    xacro_file = os.path.join(
        pkg_tracer_description, 'models/xacro', 'hunter_se.xacro')
    assert os.path.exists(
        xacro_file), "The hunter_se.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='/hunter_se',
        parameters=[robot_description_param],
    )
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py',
            ])
        ]),
        launch_arguments={
            'verbose': 'true',
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
        }.items()
    )
    spawn_hunter_se_node = Node(
        package='hunter_se_description',
        executable='spawn_hunter_se',
        namespace='/hunter_se',
        arguments=[robot_description],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/hunter_se/controller_manager"],
    )
    steering_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_trajectory_controller",
                   "--controller-manager", "/hunter_se/controller_manager"],
    )
    forward_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller",
                   "--controller-manager", "/hunter_se/controller_manager"],
    )
    ackermann_control_node = Node(
        package="hunter_se_control",
        executable="ackermann_control",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace='hunter_se'
    )

    # Launching Controllers
    ld.add_action(joint_state_broadcaster)
    ld.add_action(steering_trajectory_controller)
    ld.add_action(forward_velocity_controller)

    # Launching Gazebo
    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)

    # Spawn Robot
    ld.add_action(spawn_hunter_se_node)
    ld.add_action(robot_state_publisher_node)

    # Control Robot
    ld.add_action(joy_node)
    ld.add_action(ackermann_control_node)

    return ld
