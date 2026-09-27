"""Start Gazebo Harmonic with the SwarmX warehouse and bridge /clock.

Args:
  gui:=true|false          show the Gazebo client
  headless_rendering:=...  render sensors without a display (servers / CI / Docker)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gz(context):
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    headless = LaunchConfiguration("headless_rendering").perform(context).lower() == "true"
    gz_args = f"{world} -r" + ("" if gui else " -s") + (" --headless-rendering" if headless else "")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items())]


def generate_launch_description():
    share = get_package_share_directory("swarmx_gazebo")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=os.path.join(share, "worlds", "swarmx_warehouse.sdf")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("headless_rendering", default_value="false"),
        OpaqueFunction(function=_gz),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen"),
    ])
