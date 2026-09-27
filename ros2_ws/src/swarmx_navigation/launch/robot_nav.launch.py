"""Per-robot Nav2 stack (runs onboard each robot - nothing shared between robots).

Args
  namespace            robot namespace (robot1, ...)
  localization         amcl | static | slam_toolbox | slam_mapping
                         amcl         - particle filter on the shared map file (default)
                         static       - perfect odometry (sim only): map->odom = spawn pose
                         slam_toolbox - decentralized slam_toolbox localization on a serialized pose graph
                         slam_mapping - build the map online with slam_toolbox (map->odom from SLAM)
  executor             direct | nav2
                         direct - SwarmX drives cmd_vel (ORCA) through Nav2's velocity smoother +
                                  collision monitor (default, lightest on edge hardware)
                         nav2   - full Nav2 (planner, RPP controller, BT navigator); SwarmX releases
                                  waypoints under its zone locks and ORCA-filters the controller output
  x, y, yaw            initial pose in the map
  map                  Nav2 map yaml
  pose_graph           slam_toolbox serialized map (without extension) for localization:=slam_toolbox
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def _nav(context):
    share = get_package_share_directory("swarmx_navigation")
    ns = LaunchConfiguration("namespace").perform(context)
    loc = LaunchConfiguration("localization").perform(context)
    executor = LaunchConfiguration("executor").perform(context)
    x = float(LaunchConfiguration("x").perform(context))
    y = float(LaunchConfiguration("y").perform(context))
    yaw = float(LaunchConfiguration("yaw").perform(context))
    map_yaml = LaunchConfiguration("map").perform(context)
    pose_graph = LaunchConfiguration("pose_graph").perform(context)
    sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    loc_params = os.path.join(share, "config", "localization.yaml")
    exe_params = os.path.join(share, "config", "nav2_executor.yaml")
    common = {"use_sim_time": sim}

    nodes = [PushRosNamespace(ns), SetRemap("/tf", "tf"), SetRemap("/tf_static", "tf_static")]
    loc_nodes = []
    if loc in ("amcl", "static"):
        nodes.append(Node(package="nav2_map_server", executable="map_server", name="map_server", output="log",
                          parameters=[loc_params, common, {"yaml_filename": map_yaml}]))
        loc_nodes.append("map_server")
    if loc == "amcl":
        nodes.append(Node(package="nav2_amcl", executable="amcl", name="amcl", output="log",
                          parameters=[loc_params, common, {"initial_pose": {"x": x, "y": y, "z": 0.0, "yaw": yaw}}]))
        loc_nodes.append("amcl")
    elif loc == "static":
        # ground-truth odometry (Gazebo OdometryPublisher) is already in the world frame -> identity;
        # plain wheel odometry starts at the spawn pose -> offset by it
        world = LaunchConfiguration("odom_is_world").perform(context).lower() == "true"
        ox, oy, oyaw = (0.0, 0.0, 0.0) if world else (x, y, yaw)
        nodes.append(Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom", output="log",
                          arguments=["--x", str(ox), "--y", str(oy), "--yaw", str(oyaw),
                                     "--frame-id", "map", "--child-frame-id", "odom"],
                          parameters=[common]))
    elif loc == "slam_mapping":
        # online mapping: slam_toolbox builds the map and provides map->odom itself
        nodes.append(Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
                          output="log", parameters=[os.path.join(share, "config", "slam_mapping.yaml"), common]))
        loc_nodes.append("slam_toolbox")  # slam_toolbox is a lifecycle node in Jazzy
    elif loc == "slam_toolbox":
        slam_params = os.path.join(share, "config", "slam_localization.yaml")
        nodes.append(Node(package="slam_toolbox", executable="localization_slam_toolbox_node", name="slam_toolbox",
                          output="log", parameters=[slam_params, common,
                                                    {"map_file_name": pose_graph, "map_start_pose": [x, y, yaw]}]))
        loc_nodes.append("slam_toolbox")
    managers = []
    if loc_nodes:
        managers.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                             name="lifecycle_manager_localization", output="log",
                             parameters=[common, {"autostart": True, "node_names": loc_nodes,
                                                  # slam_toolbox does not implement Nav2 bonds
                                                  "bond_timeout": 0.0 if loc.startswith("slam") else 4.0}]))

    # safety chain: <command> -> cmd_vel_nav -> velocity_smoother -> collision_monitor -> cmd_vel
    nav_nodes = ["velocity_smoother", "collision_monitor"]
    nodes += [
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother", output="log",
             parameters=[loc_params, common], remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output="log",
             parameters=[loc_params, common]),
    ]
    if executor == "nav2":
        ctrl = [("cmd_vel", "cmd_vel_ctrl")]  # SwarmX ORCA filter: cmd_vel_ctrl -> cmd_vel_nav
        nodes += [
            Node(package="nav2_controller", executable="controller_server", output="log",
                 parameters=[exe_params, common], remappings=ctrl),
            Node(package="nav2_planner", executable="planner_server", name="planner_server", output="log",
                 parameters=[exe_params, common]),
            Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="log",
                 parameters=[exe_params, common], remappings=ctrl),
            Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="log",
                 parameters=[exe_params, common]),
            Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower",
                 output="log", parameters=[exe_params, common]),
        ]
        nav_nodes += ["controller_server", "planner_server", "behavior_server", "bt_navigator", "waypoint_follower"]
    managers.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                         name="lifecycle_manager_navigation", output="log",
                         parameters=[common, {"autostart": True, "node_names": nav_nodes}]))
    # start lifecycle managers only after the managed nodes' services have been discovered:
    # a transition request sent before discovery completes can be lost and hang bring-up
    delay = float(LaunchConfiguration("lifecycle_delay").perform(context))
    return [GroupAction(nodes), GroupAction(nodes[:3] + [TimerAction(period=delay, actions=managers)])]


def generate_launch_description():
    share = get_package_share_directory("swarmx_navigation")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot1"),
        DeclareLaunchArgument("localization", default_value="amcl"),
        DeclareLaunchArgument("executor", default_value="direct"),
        DeclareLaunchArgument("x", default_value="3.5"),
        DeclareLaunchArgument("y", default_value="6.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "warehouse.yaml")),
        DeclareLaunchArgument("pose_graph", default_value=os.path.join(share, "maps", "warehouse")),
        DeclareLaunchArgument("lifecycle_delay", default_value="4.0"),
        DeclareLaunchArgument("odom_is_world", default_value="true",
                              description="true when odom comes from the simulator's ground-truth OdometryPublisher"),
        OpaqueFunction(function=_nav),
    ])
