"""SwarmX fleet agent - the decentralized coordination brain of ONE robot, as a ROS 2 node.

Runs onboard every robot (Raspberry Pi 5 / Jetson). There is no central node:
robots coordinate only through the peer-to-peer topic ``/swarmx/p2p``.

Inputs (robot namespace):  tf (map -> base_footprint from AMCL / slam_toolbox), odom, scan
Outputs:                   cmd_vel_nav  (-> Nav2 velocity_smoother -> collision_monitor -> cmd_vel)
                           swarmx/state (typed telemetry), swarmx/markers (RViz)

Executors
  direct  SwarmX computes an ORCA-safe holonomic velocity and tracks it with the
          differential drive (NH-ORCA approximation, enlarged safety margin).
  nav2    SwarmX releases waypoints (up to its zone-lock limit) to Nav2's
          NavigateThroughPoses; Nav2 plans + tracks; SwarmX ORCA-filters the
          controller output (cmd_vel_ctrl) before the safety chain.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import ColorRGBA
from swarmx_core import orca
from swarmx_core.agent import STATIONARY, AgentConfig, FleetAgent
from swarmx_core.edge import holonomic_to_unicycle
from swarmx_core.transport import UdpMulticastTransport, ZenohTransport
from swarmx_core.warehouse import Warehouse
from swarmx_interfaces.msg import RobotState, ZoneClaim
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .perception import ScanPerception
from .ros_transport import RosP2PTransport

STATE_COLORS = {  # validated categorical slots 1-3 (dashboard palette)
    "work": (0.165, 0.471, 0.839), "carry": (0.922, 0.408, 0.204), "charge": (0.106, 0.686, 0.478),
    "idle": (0.537, 0.529, 0.506), "failed": (0.816, 0.231, 0.231),
}


def _cat(status: str) -> str:
    if status in ("to_pickup", "picking"):
        return "work"
    if status in ("to_dropoff", "dropping"):
        return "carry"
    if status in ("to_charger", "charging"):
        return "charge"
    return "failed" if status == "failed" else "idle"


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FleetAgentNode(Node):
    def __init__(self):
        super().__init__("fleet_agent")
        p = self.declare_parameter
        p("robot_id", "")
        p("transport", "ros")            # ros | udp | zenoh
        p("executor", "direct")          # direct | nav2
        p("rate", 20.0)
        p("max_speed", 0.9)
        p("safety_margin", 0.12)         # extra ORCA radius for diff-drive tracking error
        p("allocator", "cbba")
        p("motion", "swarmx")
        p("map_frame", "map")
        p("base_frame", "base_footprint")
        p("battery_topic", "")
        p("battery_init", 1.0)
        p("battery_per_m", 0.0015)
        p("use_perception", True)
        p("publish_markers", True)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        ns = self.get_namespace().strip("/")
        self.robot_id = gp("robot_id") or ns or "robot1"
        self.exec_mode = gp("executor")
        self.map_frame, self.base_frame = gp("map_frame"), gp("base_frame")
        self.dt = 1.0 / float(gp("rate"))
        kind = gp("transport")
        if kind == "udp":
            tx = UdpMulticastTransport(self.robot_id)
        elif kind == "zenoh":
            tx = ZenohTransport(self.robot_id)
        else:
            tx = RosP2PTransport(self, self.robot_id)
        cfg = AgentConfig(max_speed=float(gp("max_speed")), safety_margin=float(gp("safety_margin")),
                          allocator=gp("allocator"), motion=gp("motion"))
        self.wh = Warehouse()
        self.agent = FleetAgent(self.robot_id, self.wh, tx, cfg)
        self.perception = ScanPerception(self.wh) if gp("use_perception") else None
        self.battery = float(gp("battery_init"))
        self.battery_per_m = float(gp("battery_per_m"))
        self.external_battery = bool(gp("battery_topic"))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom = None
        self.scan = None
        self._scan_used = None
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        if self.external_battery:
            self.create_subscription(BatteryState, gp("battery_topic"), self._on_battery, 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)
        self.state_pub = self.create_publisher(RobotState, "swarmx/state", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "swarmx/markers", 5) if gp("publish_markers") else None

        self.nav_client = None
        if self.exec_mode == "nav2":
            self.nav_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
            self.create_subscription(Twist, "cmd_vel_ctrl", self._on_ctrl_cmd, 10)
            self._sent_key = None
            self._goal_handle = None
        self._last_pose = None
        self._last_state_pub = 0.0
        self._warned = 0.0
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"SwarmX agent '{self.robot_id}' up: transport={kind}, executor={self.exec_mode}")

    # ------------------------------------------------------------ inputs
    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _on_battery(self, msg: BatteryState) -> None:
        if msg.percentage == msg.percentage:  # not NaN
            self.battery = max(0.0, min(1.0, float(msg.percentage)))

    def _perceive(self, scan: LaserScan, now: float) -> None:
        """Project the scan from the lidar pose *at the scan timestamp* (TF time travel).

        Using the current pose instead would smear walls by metres while the robot
        turns, and every smear would become a phantom obstacle.
        """
        if self.odom is not None and abs(self.odom.twist.twist.angular.z) > 0.6:
            return  # mid-spin scans are unreliable on a rolling-shutter lidar
        frame = scan.header.frame_id or self.base_frame
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time.from_msg(scan.header.stamp),
                                                 timeout=Duration(seconds=0.05))
        except TransformException:
            return
        sp = (tf.transform.translation.x, tf.transform.translation.y, _yaw(tf.transform.rotation))
        peers = []
        for _, peer in self.agent.fresh_peers(now):
            m = peer.msg
            age = min(max(now - m.get("t", now), 0.0), 0.3)
            peers.append((m["x"] + m.get("vx", 0.0) * age, m["y"] + m.get("vy", 0.0) * age))
        blocked, free, bodies = self.perception.process(sp, scan.ranges, scan.angle_min, scan.angle_increment,
                                                        scan.range_min, scan.range_max, peers)
        self.agent.observe(blocked, free, robots=bodies)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    # -------------------------------------------------------------- loop
    def _tick(self) -> None:
        now = self._now()
        pose = self._pose()
        if pose is None:
            if time.monotonic() - self._warned > 5.0:
                self._warned = time.monotonic()
                self.get_logger().warn(f"waiting for TF {self.map_frame} -> {self.base_frame} (localization up?)")
            self.cmd_pub.publish(Twist())
            return
        x, y, th = pose
        v_fwd = self.odom.twist.twist.linear.x if self.odom else 0.0
        vx, vy = v_fwd * math.cos(th), v_fwd * math.sin(th)
        if self._last_pose is not None and not self.external_battery:
            self.battery = max(0.0, self.battery - self.battery_per_m * math.dist(self._last_pose[:2], (x, y)))
        self._last_pose = pose

        if self.perception is not None and self.scan is not None and self.scan is not self._scan_used:
            self._scan_used = self.scan
            self._perceive(self.scan, now)

        cmd = self.agent.step(now, self.dt, x, y, th, vx, vy, self.battery)
        for e in self.agent.events:
            self.get_logger().info(e)
        self.agent.events.clear()

        if self.exec_mode == "direct":
            v, w = holonomic_to_unicycle(cmd[0], cmd[1], th, self.agent.cfg.max_speed)
            out = Twist()
            out.linear.x, out.angular.z = float(v), float(w)
            self.cmd_pub.publish(out)
        else:
            self._nav2_update()
        if now - self._last_state_pub >= 0.2:
            self._last_state_pub = now
            self._publish_state(now)

    # ------------------------------------------------------ nav2 executor
    def _nav2_update(self) -> None:
        a = self.agent
        if a.status in STATIONARY or not a.route:
            self._cancel_goal()
            return
        lim = max(a.route_idx, min(a.last_limit, len(a.route) - 1))
        seg = a.route[a.route_idx:lim + 1]
        if not seg:
            self._cancel_goal()
            return
        key = (tuple(seg[-1]), tuple(a.route[0]))
        if key == self._sent_key:
            return
        if not self.nav_client.server_is_ready():
            return
        # compress to turning points so Nav2 plans smooth segments between SwarmX waypoints
        pts = [seg[0]]
        for i in range(1, len(seg) - 1):
            d0 = (seg[i][0] - seg[i - 1][0], seg[i][1] - seg[i - 1][1])
            d1 = (seg[i + 1][0] - seg[i][0], seg[i + 1][1] - seg[i][1])
            if d0 != d1:
                pts.append(seg[i])
        if len(seg) > 1:
            pts.append(seg[-1])
        goal = NavigateThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for i, c in enumerate(pts):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = stamp
            cx, cy = self.wh.center(c)
            nxt = pts[min(i + 1, len(pts) - 1)]
            prv = pts[max(i - 1, 0)]
            yaw = math.atan2(nxt[1] - prv[1], nxt[0] - prv[0]) if nxt != prv else 0.0
            ps.pose.position.x, ps.pose.position.y = cx, cy
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            goal.poses.append(ps)
        self._sent_key = key
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut) -> None:
        try:
            self._goal_handle = fut.result()
        except Exception:  # noqa: BLE001
            self._goal_handle = None
            self._sent_key = None

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._sent_key = None

    def _on_ctrl_cmd(self, msg: Twist) -> None:
        """ORCA filter on the Nav2 controller output (cmd_vel_ctrl -> cmd_vel_nav)."""
        pose = self._last_pose
        if pose is None:
            return
        x, y, th = pose
        a = self.agent
        v, w = msg.linear.x, msg.angular.z
        pref = (v * math.cos(th), v * math.sin(th))
        now = self._now()
        nbrs = a._neighbors(now)
        r = a.cfg.radius + a.cfg.safety_margin
        walls = self.wh.nearby_wall_points(x, y, a.cfg.wall_dist, a.planner.blocked | a._foreign_zone_cells())
        vo = orca.compute_velocity((x, y), a.vel, pref, r, a.cfg.max_speed, nbrs, walls,
                                   a.cfg.orca_horizon, a.cfg.orca_horizon_obst, self.dt)
        vo = a._emergency_filter(vo, nbrs, self.dt)
        out = Twist()
        if math.hypot(vo[0] - pref[0], vo[1] - pref[1]) < 0.05:
            out.linear.x, out.angular.z = v, w       # no conflict: keep Nav2's command
        else:
            v2, w2 = holonomic_to_unicycle(vo[0], vo[1], th, a.cfg.max_speed)
            out.linear.x, out.angular.z = float(v2), float(w2)
        self.cmd_pub.publish(out)

    # ------------------------------------------------------------ outputs
    def _publish_state(self, now: float) -> None:
        a = self.agent
        st = RobotState()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self.map_frame
        st.robot_id = self.robot_id
        st.pose.x, st.pose.y, st.pose.theta = float(a.pos[0]), float(a.pos[1]), float(a.theta)
        st.velocity = Vector3(x=float(a.vel[0]), y=float(a.vel[1]), z=0.0)
        st.battery = float(self.battery)
        st.status = a.status
        st.task = a.task or ""
        st.bundle = list(a.cbba.path)
        st.waiting_zone = a.waiting_zone or ""
        for zid, c in a.zones.mine.items():
            st.zones.append(ZoneClaim(zone=zid, mode=c.mode, state=c.state, key=float(c.key), lo=int(c.lo),
                                      hi=int(min(c.hi, 2 ** 31 - 1)), phase=c.phase, cur=int(c.cur), eta=float(c.eta)))
        st.tasks_done = a.stats.tasks_done
        st.distance_m = float(a.stats.distance)
        st.reroutes = a.stats.reroutes
        self.state_pub.publish(st)
        if self.marker_pub is not None:
            self.marker_pub.publish(self._markers())

    def _markers(self) -> MarkerArray:
        a = self.agent
        arr = MarkerArray()
        rgb = STATE_COLORS[_cat(a.status)]
        stamp = self.get_clock().now().to_msg()

        def mk(mid, mtype):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = self.robot_id
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime = Duration(seconds=1.0).to_msg()
            return m

        path = mk(0, Marker.LINE_STRIP)
        path.scale.x = 0.06
        path.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.8)
        path.points.append(Point(x=float(a.pos[0]), y=float(a.pos[1]), z=0.05))
        for c in a.route[a.route_idx:a.route_idx + 15]:
            cx, cy = self.wh.center(c)
            path.points.append(Point(x=cx, y=cy, z=0.05))
        arr.markers.append(path)
        label = mk(1, Marker.TEXT_VIEW_FACING)
        label.pose.position.x, label.pose.position.y, label.pose.position.z = float(a.pos[0]), float(a.pos[1]), 0.7
        label.scale.z = 0.3
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = f"{self.robot_id} {a.status}" + (f" [{a.task}]" if a.task else "")
        arr.markers.append(label)
        body = mk(2, Marker.CYLINDER)  # footprint, so RViz shows the whole fleet from one TF tree
        body.pose.position.x, body.pose.position.y, body.pose.position.z = float(a.pos[0]), float(a.pos[1]), 0.15
        body.scale.x = body.scale.y = 2 * a.cfg.radius
        body.scale.z = 0.3
        body.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.9)
        arr.markers.append(body)
        for i, (zid, c) in enumerate(sorted(a.zones.mine.items())):
            cells = self.wh.zones[zid].cells
            box = mk(10 + i, Marker.CUBE)
            xs = [q[0] for q in cells]
            ys = [q[1] for q in cells]
            box.pose.position.x = (min(xs) + max(xs) + 1) / 2
            box.pose.position.y = (min(ys) + max(ys) + 1) / 2
            box.pose.position.z = 0.01
            box.scale.x, box.scale.y, box.scale.z = float(max(xs) - min(xs) + 1), float(max(ys) - min(ys) + 1), 0.02
            alpha = 0.45 if c.state == "hold" else 0.15
            box.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=alpha)
            arr.markers.append(box)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = FleetAgentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
