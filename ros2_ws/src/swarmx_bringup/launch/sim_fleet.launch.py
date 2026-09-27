"""One command: Gazebo Harmonic warehouse + N robots (each with its own Nav2 stack and
SwarmX agent) + task source + dashboard + RViz.

    ros2 launch swarmx_bringup sim_fleet.launch.py robots:=3
    ros2 launch swarmx_bringup sim_fleet.launch.py robots:=5 localization:=static gui:=false
    ros2 launch swarmx_bringup sim_fleet.launch.py executor:=nav2 scenario:=crossing

Run with rmw_zenoh in router-less peer mode for true decentralization:
    source $(ros2 pkg prefix swarmx_bringup)/share/swarmx_bringup/config/zenoh/swarmx_zenoh.env
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from swarmx_core.warehouse import Warehouse


def _fleet(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    n = int(lc("robots"))
    localization, executor = lc("localization"), lc("executor")
    spawn_delay = float(lc("spawn_delay"))
    gz = get_package_share_directory("swarmx_gazebo")
    nav = get_package_share_directory("swarmx_navigation")
    fleet = get_package_share_directory("swarmx_fleet")
    bringup = get_package_share_directory("swarmx_bringup")
    wh = Warehouse()
    actions = []
    if os.environ.get("RMW_IMPLEMENTATION") == "rmw_zenoh_cpp" and lc("zenoh_router") == "true":
        # one Zenoh router per host: every process on this machine attaches to it, and routers on
        # different robots discover each other (see config/zenoh/README.md). On a single simulation
        # host it stands in for "each robot's local router".
        actions.append(Node(package="rmw_zenoh_cpp", executable="rmw_zenohd", name="zenoh_router", output="log"))
    actions += [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, "launch", "gazebo.launch.py")),
        launch_arguments={"gui": lc("gui"), "headless_rendering": lc("headless_rendering")}.items())]
    for i in range(n):
        ns = f"robot{i + 1}"
        x, y = wh.center(wh.depot[i % len(wh.depot)])
        per_robot = [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz, "launch", "spawn_robot.launch.py")),
                                     launch_arguments={"namespace": ns, "x": str(x), "y": str(y), "yaw": "0.0"}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav, "launch", "robot_nav.launch.py")),
                                     launch_arguments={"namespace": ns, "localization": localization, "executor": executor,
                                                       "x": str(x), "y": str(y), "yaw": "0.0"}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(fleet, "launch", "fleet_agent.launch.py")),
                                     launch_arguments={"namespace": ns, "executor": executor,
                                                       "max_speed": lc("max_speed")}.items()),
        ]
        actions.append(TimerAction(period=3.0 + i * spawn_delay, actions=per_robot))
    actions.append(TimerAction(period=3.0 + n * spawn_delay + 2.0, actions=[
        Node(package="swarmx_fleet", executable="task_source", name="task_source", output="screen",
             parameters=[{"use_sim_time": True, "tasks": int(lc("tasks")), "scenario": lc("scenario"),
                          "stream_per_min": float(lc("stream_per_min")), "start_delay": 10.0}]),
    ]))
    actions.append(Node(package="swarmx_dashboard", executable="dashboard", name="swarmx_dashboard", output="screen",
                        condition=IfCondition(lc("dashboard")),
                        parameters=[{"use_sim_time": True, "port": int(lc("port")), "gazebo_world": "swarmx_warehouse"}]))
    actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                        condition=IfCondition(lc("rviz")),
                        arguments=["-d", os.path.join(bringup, "rviz", "fleet.rviz")],
                        remappings=[("/tf", "/robot1/tf"), ("/tf_static", "/robot1/tf_static")],
                        parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="3"),
        DeclareLaunchArgument("localization", default_value="amcl", description="amcl | static | slam_toolbox"),
        DeclareLaunchArgument("executor", default_value="direct", description="direct | nav2"),
        DeclareLaunchArgument("scenario", default_value="random", description="random | crossing | hot_aisles"),
        DeclareLaunchArgument("tasks", default_value="12"),
        DeclareLaunchArgument("stream_per_min", default_value="0.0"),
        DeclareLaunchArgument("max_speed", default_value="0.9"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("headless_rendering", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("dashboard", default_value="true"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("spawn_delay", default_value="2.0"),
        DeclareLaunchArgument("zenoh_router", default_value="true"),
        OpaqueFunction(function=_fleet),
    ])
