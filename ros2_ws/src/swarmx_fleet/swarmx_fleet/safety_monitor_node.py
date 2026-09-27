"""Ground-truth safety monitor for simulation (NOT part of the robots' software).

Subscribes to every robot's ground-truth odometry (Gazebo OdometryPublisher),
tracks the closest centre-to-centre distance and counts contact events
(distance < 2 x body radius). Publishes a JSON summary on /swarmx/safety,
which the dashboard shows in its collision tile.
"""
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")
        self.declare_parameter("robots", 3)
        self.declare_parameter("body_radius", 0.28)
        n = int(self.get_parameter("robots").value)
        self.r = float(self.get_parameter("body_radius").value)
        self.pos = {}
        for i in range(1, n + 1):
            ns = f"robot{i}"
            self.create_subscription(Odometry, f"/{ns}/odom", lambda m, ns=ns: self._on_odom(ns, m), 10)
        self.pub = self.create_publisher(String, "/swarmx/safety", 10)
        self.min_sep = math.inf
        self.contacts = 0
        self.in_contact = set()
        self.create_timer(0.05, self._check)
        self.create_timer(1.0, self._publish)

    def _on_odom(self, ns, m):
        p = m.pose.pose.position
        self.pos[ns] = (p.x, p.y)

    def _check(self):
        ids = sorted(self.pos)
        now = set()
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                d = math.dist(self.pos[ids[i]], self.pos[ids[j]])
                self.min_sep = min(self.min_sep, d)
                if d < 2 * self.r - 0.005:
                    now.add((ids[i], ids[j]))
        for pair in now - self.in_contact:
            self.contacts += 1
            self.get_logger().error(f"CONTACT {pair[0]} <-> {pair[1]}")
        self.in_contact = now

    def _publish(self):
        self.pub.publish(String(data=json.dumps({
            "collisions": self.contacts,
            "min_separation": round(self.min_sep, 3) if self.min_sep < math.inf else None,
            "robots_tracked": len(self.pos)})))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
