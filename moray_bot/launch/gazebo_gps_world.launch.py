# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    moray_bot_dir = get_package_share_directory(
        "moray_bot")
    launch_dir = os.path.join(moray_bot_dir, 'launch')
    world = os.path.join(moray_bot_dir, "worlds", "sonoma_raceway.world")

    urdf = os.path.join(moray_bot_dir, 'urdf', 'turtlebot3_waffle_gps.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    models_dir = os.path.join(moray_bot_dir, "models")
    models_dir += os.pathsep + \
        f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    set_gazebo_model_path_cmd = None

    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + \
            os.pathsep + models_dir
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", gazebo_model_path)
    else:
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", models_dir)

    set_tb3_model_cmd = SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle")

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True} ])
    
    # RVIZ
    rviz_config_file = os.path.join(moray_bot_dir, "rviz", "rviz.rviz")
    # start_rviz_cmd = ExecuteProcess(
    #     cmd=['rviz2', '-d', rviz_config_file],
    #     cwd=[launch_dir], output='both')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}])

    # Map transform
    map_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_transform',
        output='both',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])
    
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set gazebo up to find models properly
    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_tb3_model_cmd)

    # simulator launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)

    # rviz launch
    ld.add_action(start_rviz_cmd)

    # map transform launch
    ld.add_action(map_transform_cmd)

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    return ld
