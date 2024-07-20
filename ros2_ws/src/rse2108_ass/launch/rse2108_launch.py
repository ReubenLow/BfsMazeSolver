import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    turtlebot_dir = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    
    #Get the parameters file directory
    mark_dir = get_package_share_directory('rse2108_ass')
    params_file = os.path.join(mark_dir, 'params', 'nav_params.yaml')
    slam_param_file = os.path.join(mark_dir, 'params', 'slam_params.yaml')


    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': slam_param_file}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'params_file': params_file,
                              'container_name': 'nav2_container'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(turtlebot_dir, 'launch', 'simplemaze2.launch.py')),
            launch_arguments={}.items()),


    ])



    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(bringup_cmd_group)
    return ld

# declare_slam_params_file_cmd = DeclareLaunchArgument(
#         'slam_params_file',
#         default_value=os.path.join(get_package_share_directory("slam_toolbox"),
#                                    'config', 'mapper_params_online_async.yaml'),
#         description='Full path to the ROS2 parameters file to use for the slam_toolbox node')    
