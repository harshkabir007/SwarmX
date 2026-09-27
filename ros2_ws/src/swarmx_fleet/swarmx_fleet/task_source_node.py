"""Warehouse-management-system peer: injects transport tasks into the swarm.

It is *not* a planner - it only announces what needs moving (and re-announces
until someone reports delivery). Allocation happens among the robots (CBBA).

Parameters
  tasks         initial batch size (default 12)
  stream_per_min  new tasks per minute after the batch (0 = batch only)
  scenario      random | crossing | hot_aisles
  seed          RNG seed
"""
import json
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swarmx_core import protocol
from swarmx_core.warehouse import Warehouse

from .ros_transport import RosP2PTransport


class TaskSourceNode(Node):
    def __init__(self):
        super().__init__("task_source")
        self.declare_parameter("tasks", 12)
        self.declare_parameter("stream_per_min", 0.0)
        self.declare_parameter("scenario", "random")
        self.declare_parameter("seed", 1)
        self.declare_parameter("start_delay", 5.0)
        self.wh = Warehouse()
        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.scenario = self.get_parameter("scenario").value
        self.tx = RosP2PTransport(self, "wms")
        self.status_pub = self.create_publisher(String, "/swarmx/wms/status", 10)
        self.tasks, self.done = {}, {}
        self.n = 0
        self.t0 = None
        self._pending_batch = int(self.get_parameter("tasks").value)
        self._stream = float(self.get_parameter("stream_per_min").value)
        self._delay = float(self.get_parameter("start_delay").value)
        self._next_stream = None
        self.create_timer(0.2, self._poll)
        self.create_timer(2.0, self._announce)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _make(self, t):
        self.n += 1
        wh = self.wh
        if self.scenario == "crossing":
            x = self.rng.choice(wh.aisle_xs[2:6])
            s, n = (x, self.rng.choice((2, 3))), (x, self.rng.choice((16, 17)))
            pick, drop = (s, n) if self.n % 2 else (n, s)
        elif self.scenario == "hot_aisles":
            pick = self.rng.choice([c for c in wh.pickups if c[0] in wh.aisle_xs[2:5]])
            drop = self.rng.choice(wh.dropoffs)
        else:
            pick, drop = self.rng.choice(wh.pickups), self.rng.choice(wh.dropoffs)
        task = {"id": f"T{self.n:03d}", "pickup": list(pick), "dropoff": list(drop), "created": round(t, 2), "priority": 1.0}
        self.tasks[task["id"]] = task
        return task

    def _poll(self):
        now = self._now()
        if self.t0 is None:
            self.t0 = now
        if self._pending_batch and now - self.t0 >= self._delay:
            batch = [self._make(now) for _ in range(self._pending_batch)]
            self._pending_batch = 0
            self.tx.publish(protocol.make(protocol.TASK, "wms", now, tasks=batch))
            self.get_logger().info(f"announced {len(batch)} tasks")
            self._next_stream = now
        if self._stream > 0 and self._next_stream is not None and now >= self._next_stream + 60.0 / self._stream:
            self._next_stream = now
            self.tx.publish(protocol.make(protocol.TASK, "wms", now, tasks=[self._make(now)]))
        for m in self.tx.poll():
            if m.get("type") == protocol.TASK_DONE and m["task"] in self.tasks and m["task"] not in self.done:
                self.done[m["task"]] = (now, m.get("src"))
                self.get_logger().info(f"{m['task']} delivered by {m.get('src')} ({len(self.done)}/{len(self.tasks)})")
            elif m.get("type") == protocol.CBBA:
                for tid in m.get("done", ()):
                    if tid in self.tasks and tid not in self.done:
                        self.done[tid] = (now, None)

    def _announce(self):
        now = self._now()
        pending = [t for tid, t in self.tasks.items() if tid not in self.done]
        if pending:
            self.tx.publish(protocol.make(protocol.TASK, "wms", now, tasks=pending))
        comp = [self.done[t][0] - self.tasks[t]["created"] for t in self.done]
        status = {"created": len(self.tasks), "delivered": len(self.done), "pending": len(pending),
                  "mean_completion_s": round(sum(comp) / len(comp), 1) if comp else None,
                  "makespan_s": round(max(d[0] for d in self.done.values()) - min(t["created"] for t in self.tasks.values()), 1)
                  if self.done and not pending else None}
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = TaskSourceNode()
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
