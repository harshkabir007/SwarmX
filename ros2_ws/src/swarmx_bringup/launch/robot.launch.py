"""Bring up SwarmX on ONE physical robot (e.g. Raspberry Pi 5).

Your base driver must provide, in the robot namespace: odom (nav_msgs/Odometry),
tf odom->base_footprint, scan (sensor_msgs/LaserScan) and accept cmd_vel
(geometry_msgs/Twist). Robot description / static TFs come from swarmx_description
or your own URDF.

    source .../swarmx_zenoh.env        # router-less peer-to-peer DDS replacement
    ros2 launch swarmx_bringup robot.launch.py namespace:=robot2 x:=4.5 y:=6.5
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lc = LaunchConfiguration
    nav = get_package_share_directory("swarmx_navigation")
    fleet = get_package_share_directory("swarmx_fleet")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot1"),
        DeclareLaunchArgument("x", default_value="3.5"),
        DeclareLaunchArgument("y", default_value="6.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("localization", default_value="amcl"),
        DeclareLaunchArgument("executor", default_value="direct"),
        DeclareLaunchArgument("max_speed", default_value="0.6"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav, "launch", "robot_nav.launch.py")),
                                 launch_arguments={"namespace": lc("namespace"), "localization": lc("localization"),
                                                   "executor": lc("executor"), "x": lc("x"), "y": lc("y"),
                                                   "yaw": lc("yaw"), "use_sim_time": "false"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(fleet, "launch", "fleet_agent.launch.py")),
                                 launch_arguments={"namespace": lc("namespace"), "executor": lc("executor"),
                                                   "use_sim_time": "false", "max_speed": lc("max_speed")}.items()),
    ])
