"""SwarmX fleet dashboard for ROS 2 - a *passive peer* on /swarmx/p2p.

Serves the web UI (http://<host>:8080) plus REST (/api/snapshot, /api/cmd) and
a WebSocket (/ws). It never commands motion: operator actions are published
as ordinary P2P messages that robots are free to act on (new tasks, a
reported obstacle, a failure-injection command for demos).

Parameters
  port, host             web server bind
  gazebo_world           if set, "place obstacle" spawns a physical pallet in Gazebo
                         (robots must discover it with their own lidar); otherwise a
                         virtual obstacle report is broadcast
"""
import asyncio
import random
import subprocess
import threading

import rclpy
from rclpy.node import Node
from swarmx_core import protocol
from swarmx_core.dashboard.monitor import FleetMonitor
from swarmx_core.dashboard.server import DashboardServer
from swarmx_core.warehouse import Warehouse
from swarmx_fleet.ros_transport import RosP2PTransport


class DashboardNode(Node):
    def __init__(self):
        super().__init__("swarmx_dashboard")
        self.declare_parameter("port", 8080)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("gazebo_world", "")
        self.wh = Warehouse()
        self.tx = RosP2PTransport(self, "dashboard")
        self.monitor = FleetMonitor()
        self.world_name = self.get_parameter("gazebo_world").value
        self.virtual = set()
        self.physical = set()
        self.n_tasks = 0
        self.create_timer(0.05, self._pump)
        self.create_timer(8.0, self._refresh_virtual)
        port = int(self.get_parameter("port").value)
        host = self.get_parameter("host").value
        controls = ["add_tasks", "block", "fail"]
        self.server = DashboardServer(self._world, self._snapshot, self._command, host=host, port=port,
                                      info={"source": "ros", "controls": controls})
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._serve, daemon=True).start()
        self.get_logger().info(f"SwarmX dashboard on http://localhost:{port}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _serve(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self.server.start())
        self._loop.run_forever()

    def _pump(self) -> None:
        now = self._now()
        for m in self.tx.poll():
            self.monitor.ingest(m, now=now)

    def _world(self) -> dict:
        w = self.wh.to_dict()
        w["pickups"] = self.wh.pickups
        return w

    def _snapshot(self) -> dict:
        return self.monitor.snapshot(now=self._now())

    # --------------------------------------------------------- commands
    def _command(self, cmd: dict) -> dict:
        c = cmd.get("cmd")
        now = self._now()
        if c == "add_tasks":
            batch = []
            for _ in range(int(cmd.get("n", 5))):
                self.n_tasks += 1
                batch.append({"id": f"D{self.n_tasks:03d}", "pickup": list(random.choice(self.wh.pickups)),
                              "dropoff": list(random.choice(self.wh.dropoffs)), "created": round(now, 2),
                              "priority": 1.0})
            self.tx.publish(protocol.make(protocol.TASK, "dashboard", now, tasks=batch))
            self.monitor.note(f"Operator added {len(batch)} tasks", now=now)
            return {"ok": True}
        if c == "toggle_block":
            cell = tuple(int(v) for v in cmd["cell"])
            if not self.wh.is_free(cell):
                return {"ok": False, "error": "not a floor cell"}
            if self.world_name:
                return self._toggle_physical(cell, now)
            on = cell not in self.virtual
            (self.virtual.add if on else self.virtual.discard)(cell)
            msg = protocol.make(protocol.BLOCKED, "dashboard", now, cells=[list(cell)] if on else [],
                                cleared=[] if on else [list(cell)])
            self.tx.publish(msg)
            self.monitor.ingest(msg, now=now)
            return {"ok": True}
        if c in ("fail", "recover"):
            self.tx.publish(protocol.make(protocol.CMD, "dashboard", now, target=cmd["robot"], cmd=c))
            self.monitor.note(f"Operator: {c} {cmd['robot']}", kind="alert", now=now)
            return {"ok": True}
        return {"ok": False, "error": f"unsupported command {c!r}"}

    def _refresh_virtual(self) -> None:
        if self.virtual:
            msg = protocol.make(protocol.BLOCKED, "dashboard", self._now(), cells=[list(c) for c in self.virtual], cleared=[])
            self.tx.publish(msg)

    def _toggle_physical(self, cell, now) -> dict:
        name = f"pallet_{cell[0]}_{cell[1]}"
        x, y = self.wh.center(cell)
        if cell in self.physical:
            req = f'name: "{name}" type: MODEL'
            ok = self._gz_service(f"/world/{self.world_name}/remove", "gz.msgs.Entity", req)
            if ok:
                self.physical.discard(cell)
                self.monitor.note(f"Pallet removed at {cell}", kind="info", now=now)
        else:
            sdf = (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>'
                   f'<pose>{x} {y} 0.4 0 0 0</pose><link name="link">'
                   '<collision name="c"><geometry><box><size>0.8 0.8 0.8</size></box></geometry></collision>'
                   '<visual name="v"><geometry><box><size>0.8 0.8 0.8</size></box></geometry>'
                   '<material><ambient>0.55 0.35 0.15 1</ambient><diffuse>0.55 0.35 0.15 1</diffuse></material>'
                   '</visual></link></model></sdf>')
            req = 'sdf: "' + sdf.replace('"', '\\"') + '"'
            ok = self._gz_service(f"/world/{self.world_name}/create", "gz.msgs.EntityFactory", req)
            if ok:
                self.physical.add(cell)
                self.monitor.note(f"Pallet dropped at {cell}; robots must detect it with lidar", kind="alert", now=now)
        return {"ok": ok} if ok else {"ok": False, "error": "gz service call failed"}

    def _gz_service(self, service: str, reqtype: str, req: str) -> bool:
        try:
            out = subprocess.run(["gz", "service", "-s", service, "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
                                  "--timeout", "3000", "--req", req], capture_output=True, text=True, timeout=6)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(f"gz service failed: {exc}")
            return False
        return "true" in out.stdout


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
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
