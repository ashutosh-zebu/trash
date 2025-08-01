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
    robot_description_pkg = "robot_description"

    current_pkg = get_package_share_directory("drone_nav_bringup")
    path_planner_pkg = get_package_share_directory("drone_nav_path_planner")
    behavior_tree_pkg = get_package_share_directory("drone_nav_bt")
    controller_pkg = get_package_share_directory("drone_nav_controller")

    config = os.path.join(current_pkg, 'params','nav_pipeline_params.yaml')
    
    try:
        static_tf = Node(
            package= robot_description_pkg,
            executable="tf_pub.py"
        )
    except Exception as e:
        print(f"error : {e}")



    # https://github.com/TixiaoShan/LIO-SAM.git
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lio_sam'),
                'launch',
                'run.launch.py'
            ])
        ])
    )

    try:
        global_voxel = Node(
            package= pkg,
            executable="voxel"
        )
    except Exception as e:
        print(f"error : {e}")

    try:
        local_voxel = Node(
            package= pkg,
            executable="local_voxel",
            name="local_occupancy_grid_server",
            parameters=[config],
            output="screen"
        )
    except Exception as e:
        print(f"error : {e}")

    try:
        local_path_planner = Node(
            package= pkg,
            executable="local_planner",
            name="local_planner",
            parameters=[config],
            output="screen"
        )
    except Exception as e:
        print(f"error : {e}")

    try:
        global_path_planner = Node(
            package= pkg,
            executable="path",
            name="global_planner",
            parameters=[config]
        )
    except Exception as e:
        print(f"error : {e}")
    
    
    return LaunchDescription([
        static_tf,
        # lio_sam_launch,     # The lidar inertial odometry slam. Uncomment if using this slam technique.
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
            arguments=['-d', current_pkg+"/rviz"+"/path_planner.rviz"]
        ),
    ])