# Copyright (c) 2021 Samsung Research America
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    #package/World/map/<map_name>.yaml OMIT
    #warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world') OMIT
    warehouse_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    #map_yaml_file = os.path.join(warehouse_dir, 'maps', '005', 'map.yaml') OMIT
    
    #Retrieve the Navigation Stack 2 Directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    #------------------------------ ?
    maze_dir = get_package_share_directory('turtlebot3_gazebo', 'worlds')
    world = os.path.join(maze_dir, 'turtlebot3_world.world')
    print("Test 1 clear")

#######################################Tweak to original gazebo############################### 
    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    # Declare the launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    # start the simulation
    # start_gazebo_server_cmd = ExecuteProcess(
    #     cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world],
    #     cwd=[warehouse_dir], output='screen')

    # start_gazebo_client_cmd = ExecuteProcess(
    #     condition=IfCondition(PythonExpression(['not ', headless])),
    #     cmd=['gzclient'],
    #     cwd=[warehouse_dir], output='screen')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
####################################Ignore/copyfiles####################################


    #Turtlebot3 Waffle Change Model
    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf])


#######################################Ignore/copyfiles############################### 
    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())

####################################Enable SLAM / Navigation simultaneous while mapping################################
    #Enable SLAM
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')), launch_arguments={'slam': 'True', 'map':''}.items())

    print("Test 2 clear")
        #,launch_arguments={'map': map_yaml_file}.items()) OMIT

    # start the demo autonomy task
    demo_cmd = Node(
        package='nav2_simple_commander',
        executable='example_nav_to_pose',
        emulate_tty=True,
        output='screen')


#######################################Ignore/copyfiles############################### 
    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    # ld.add_action(bringup_cmd)
    # ld.add_action(demo_cmd)
    return ld
#####################################################################################
