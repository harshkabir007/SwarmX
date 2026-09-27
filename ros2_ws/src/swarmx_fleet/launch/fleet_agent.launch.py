"""Start the SwarmX agent for one robot (onboard computer)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot1"),
        DeclareLaunchArgument("executor", default_value="direct"),
        DeclareLaunchArgument("transport", default_value="ros"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("max_speed", default_value="0.9"),
        Node(package="swarmx_fleet", executable="fleet_agent", name="fleet_agent", namespace=lc("namespace"),
             output="screen", remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
             parameters=[{"executor": lc("executor"), "transport": lc("transport"),
                          "use_sim_time": lc("use_sim_time"), "max_speed": lc("max_speed")}]),
    ])
