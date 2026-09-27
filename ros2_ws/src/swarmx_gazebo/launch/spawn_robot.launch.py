"""Spawn one namespaced SwarmX AMR into the running Gazebo world.

Everything the robot publishes lives under /<namespace>/..., including its
TF tree (/<namespace>/tf), so any number of robots can share the world with
no central node - exactly as they would on separate computers.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def _spawn(context):
    ns = LaunchConfiguration("namespace").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    xacro_file = os.path.join(get_package_share_directory("swarmx_description"), "urdf", "swarmx_amr.urdf.xacro")
    description = ParameterValue(Command(["xacro ", xacro_file, f" namespace:={ns}"]), value_type=str)
    bridge_topics = [
        f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        f"/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        f"/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        f"/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=ns,
             parameters=[{"robot_description": description, "use_sim_time": use_sim_time}],
             remappings=[("/tf", "tf"), ("/tf_static", "tf_static")], output="log"),
        Node(package="ros_gz_sim", executable="create", name=f"spawn_{ns}", output="log",
             arguments=["-name", ns, "-topic", f"/{ns}/robot_description",
                        "-x", x, "-y", y, "-z", "0.02", "-Y", yaw]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name=f"{ns}_gz_bridge",
             arguments=bridge_topics, parameters=[{"use_sim_time": use_sim_time}], output="log"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot1"),
        DeclareLaunchArgument("x", default_value="3.5"),
        DeclareLaunchArgument("y", default_value="6.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_spawn),
    ])
