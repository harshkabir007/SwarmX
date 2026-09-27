"""SwarmX P2P transport over a ROS 2 topic.

All robots publish and subscribe on ``/swarmx/p2p``. Run with rmw_zenoh in
peer mode (multicast scouting, no router) and this is a fully decentralized
bus - see swarmx_bringup/config/zenoh/README.md.
"""
import json
import threading
from collections import deque

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from swarmx_core.transport import Transport
from swarmx_interfaces.msg import P2P

P2P_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)


class RosP2PTransport(Transport):
    def __init__(self, node, node_id: str, topic: str = "/swarmx/p2p"):
        self.node = node
        self.node_id = node_id
        self.pub = node.create_publisher(P2P, topic, P2P_QOS)
        self.sub = node.create_subscription(P2P, topic, self._on_msg, P2P_QOS)
        self._inbox = deque(maxlen=5000)
        self._lock = threading.Lock()
        self.rx = 0
        self.tx = 0

    def _on_msg(self, m: P2P) -> None:
        if m.src == self.node_id:
            return
        try:
            msg = json.loads(m.payload)
        except ValueError:
            return
        if not isinstance(msg, dict):
            return
        msg.setdefault("src", m.src)
        msg.setdefault("type", m.type)
        with self._lock:
            self._inbox.append(msg)
            self.rx += 1

    def publish(self, msg: dict) -> None:
        out = P2P()
        out.src = str(msg.get("src", self.node_id))
        out.type = str(msg.get("type", ""))
        out.stamp = float(msg.get("t", 0.0))
        out.payload = json.dumps(msg, separators=(",", ":"))
        self.pub.publish(out)
        self.tx += 1

    def poll(self):
        with self._lock:
            out = list(self._inbox)
            self._inbox.clear()
        return out
