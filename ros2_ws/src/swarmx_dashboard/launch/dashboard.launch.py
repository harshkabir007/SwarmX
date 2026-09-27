from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("gazebo_world", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="swarmx_dashboard", executable="dashboard", name="swarmx_dashboard", output="screen",
             parameters=[{"port": LaunchConfiguration("port"), "gazebo_world": LaunchConfiguration("gazebo_world"),
                          "use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
