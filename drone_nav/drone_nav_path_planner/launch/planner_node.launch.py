from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = "drone_nav_path_planner"
    planner_pkg = get_package_share_directory("drone_nav_path_planner")
    print(planner_pkg+"/rviz"+"/path_planner.rviz")
    try:
        global_voxel = Node(
            package= pkg,
            executable="voxel"
        )
    except:
        print("issue in global_voxel")

    try:
        local_voxel = Node(
            package= pkg,
            executable="local_voxel"
        )
    except:
        print("issue in local_voxel")

    try:
        global_path_planner = Node(
            package= pkg,
            executable="path"
        )
    except:
        print("issue in path")

    try:
        local_path_planner = Node(
            package= pkg,
            executable="local_planner"
        )
    except:
        print("issue in path")
    


    return LaunchDescription([
        global_voxel,
        local_voxel,
        global_path_planner,
        local_path_planner,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],  # or True if using simulation
            arguments=['-d', planner_pkg+"/rviz"+"/path_planner.rviz"]
        ),
    ])