"""Build a map with slam_toolbox using one robot (drive it with teleop or let SwarmX tasks move it).

    ros2 launch swarmx_navigation slam_mapping.launch.py namespace:=robot1
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel
    ros2 run nav2_map_server map_saver_cli -f ~/warehouse --ros-args -r map:=/robot1/map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def _slam(context):
    ns = LaunchConfiguration("namespace").perform(context)
    sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    params = os.path.join(get_package_share_directory("swarmx_navigation"), "config", "slam_mapping.yaml")
    return [GroupAction([
        PushRosNamespace(ns), SetRemap("/tf", "tf"), SetRemap("/tf_static", "tf_static"),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox", output="screen",
             parameters=[params, {"use_sim_time": sim}]),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_slam),
    ])
