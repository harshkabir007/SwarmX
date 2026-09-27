"""SwarmX edge runtime - one robot, no ROS required (Raspberry Pi 5 / Jetson).

Runs a :class:`FleetAgent` against a real (or mock) differential-drive base
and talks to the rest of the fleet over UDP multicast or Eclipse Zenoh.

    # three terminals (or three Pis on the same Wi-Fi) form a real P2P fleet:
    python -m swarmx_core.edge --id robot1 --start 3 6
    python -m swarmx_core.edge --id robot2 --start 4 6
    python -m swarmx_core.edge --id robot3 --start 3 13
    # a task source and a dashboard are just more peers:
    python -m swarmx_core.edge --wms --tasks 12
    python -m swarmx_core.edge --dashboard --port 8080

Write a driver for your motor controller by subclassing :class:`BaseDriver`
(``read_pose``/``command``); :class:`MockBase` integrates kinematics so the
whole stack can be exercised without hardware.
"""
from __future__ import annotations

import argparse
import asyncio
import math
import random
import sys
import time
from typing import Optional, Tuple

from . import protocol
from .agent import AgentConfig, FleetAgent
from .transport import Transport, UdpMulticastTransport, ZenohTransport
from .warehouse import Warehouse


class BaseDriver:
    """Interface to the robot base. Implement for real hardware."""

    def read_pose(self) -> Tuple[float, float, float, float, float]:
        """Return (x, y, theta, vx, vy) in the warehouse (map) frame."""
        raise NotImplementedError

    def command(self, v: float, w: float, dt: float) -> None:
        """Apply linear velocity v [m/s] and angular velocity w [rad/s]."""
        raise NotImplementedError

    def battery(self) -> float:
        return 1.0


class MockBase(BaseDriver):
    """Unicycle kinematics with acceleration limits (bench testing)."""

    def __init__(self, x: float, y: float, th: float = 0.0, a_max: float = 1.5):
        self.x, self.y, self.th = x, y, th
        self.v = self.w = 0.0
        self.a_max = a_max
        self.bat = 1.0

    def read_pose(self):
        return self.x, self.y, self.th, self.v * math.cos(self.th), self.v * math.sin(self.th)

    def command(self, v: float, w: float, dt: float) -> None:
        dv = max(-self.a_max * dt, min(self.a_max * dt, v - self.v))
        self.v += dv
        self.w = w
        self.th = math.atan2(math.sin(self.th + self.w * dt), math.cos(self.th + self.w * dt))
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        self.bat = max(0.0, self.bat - 0.0015 * abs(self.v) * dt - 0.00005 * dt)

    def battery(self) -> float:
        return self.bat


def holonomic_to_unicycle(vx: float, vy: float, theta: float, v_max: float = 1.0,
                          w_max: float = 2.0, k_w: float = 3.0) -> Tuple[float, float]:
    """Track an ORCA (holonomic) velocity with a differential-drive base.

    Turn in place when the heading error exceeds 90 deg, otherwise drive at the
    projected speed while steering proportionally. The residual tracking error
    is covered by the enlarged ORCA radius (``safety_margin``) - the standard
    NH-ORCA approximation.
    """
    sp = math.hypot(vx, vy)
    if sp < 0.02:
        return 0.0, 0.0
    err = math.atan2(math.sin(math.atan2(vy, vx) - theta), math.cos(math.atan2(vy, vx) - theta))
    w = max(-w_max, min(w_max, k_w * err))
    if abs(err) > math.pi / 2:
        return 0.0, w
    return min(v_max, sp * math.cos(err)), w


def make_transport(kind: str, node_id: str, zenoh_connect=None) -> Transport:
    if kind == "zenoh":
        return ZenohTransport(node_id, connect=zenoh_connect)
    return UdpMulticastTransport(node_id)


async def run_robot(args) -> None:
    wh = Warehouse()
    tx = make_transport(args.transport, args.id, args.connect)
    x, y = args.start[0] + 0.5, args.start[1] + 0.5
    base: BaseDriver = MockBase(x, y)
    cfg = AgentConfig(safety_margin=0.1)  # extra margin for diff-drive tracking error
    agent = FleetAgent(args.id, wh, tx, cfg)
    agent.pos = (x, y)
    dt = 1.0 / args.rate
    t0 = time.monotonic()
    print(f"[{args.id}] up on {args.transport}; Ctrl+C to stop", file=sys.stderr)
    while True:
        t = time.monotonic() - t0
        px, py, th, vx, vy = base.read_pose()
        cmd = agent.step(t, dt, px, py, th, vx, vy, base.battery())
        v, w = holonomic_to_unicycle(cmd[0], cmd[1], th, cfg.max_speed)
        base.command(v, w, dt)
        for e in agent.events:
            print(f"[{args.id} {t:7.1f}s] {e}", file=sys.stderr)
        agent.events.clear()
        await asyncio.sleep(max(0.0, dt - ((time.monotonic() - t0) - t)))


async def run_wms(args) -> None:
    """A task source peer (stand-in for the warehouse management system)."""
    wh = Warehouse()
    tx = make_transport(args.transport, "wms", args.connect)
    rng = random.Random(args.seed)
    tasks = []
    for i in range(args.tasks):
        tasks.append({"id": f"T{i + 1:03d}", "pickup": list(rng.choice(wh.pickups)),
                      "dropoff": list(rng.choice(wh.dropoffs)), "created": 0.0, "priority": 1.0})
    done = set()
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        for m in tx.poll():
            if m.get("type") == protocol.TASK_DONE:
                done.add(m["task"])
            elif m.get("type") == protocol.CBBA:
                done |= set(m.get("done", ()))
        pending = [tk for tk in tasks if tk["id"] not in done]
        tx.publish(protocol.make(protocol.TASK, "wms", t, tasks=pending))
        print(f"[wms {t:6.1f}s] {len(done)}/{len(tasks)} delivered", file=sys.stderr)
        await asyncio.sleep(2.0)


async def run_dashboard(args) -> None:
    from .dashboard.monitor import FleetMonitor
    from .dashboard.server import DashboardServer
    wh = Warehouse()
    tx = make_transport(args.transport, "dashboard", args.connect)
    mon = FleetMonitor()
    wms_n = [0]

    def command(cmd: dict) -> dict:
        c = cmd.get("cmd")
        now = time.monotonic()
        if c == "add_tasks":
            rng = random.Random()
            batch = []
            for _ in range(int(cmd.get("n", 5))):
                wms_n[0] += 1
                batch.append({"id": f"D{wms_n[0]:03d}", "pickup": list(rng.choice(wh.pickups)),
                              "dropoff": list(rng.choice(wh.dropoffs)), "created": 0.0, "priority": 1.0})
            tx.publish(protocol.make(protocol.TASK, "dashboard", now, tasks=batch))
            return {"ok": True}
        if c == "toggle_block":
            cell = list(cmd["cell"])
            on = tuple(cell) not in mon.blocked
            tx.publish(protocol.make(protocol.BLOCKED, "dashboard", now, cells=[cell] if on else [],
                                     cleared=[] if on else [cell]))
            mon.ingest(protocol.make(protocol.BLOCKED, "dashboard", now, cells=[cell] if on else [],
                                     cleared=[] if on else [cell]), now=now)
            return {"ok": True}
        if c in ("fail", "recover"):
            tx.publish(protocol.make(protocol.CMD, "dashboard", now, target=cmd["robot"], cmd=c))
            return {"ok": True}
        return {"ok": False, "error": f"unsupported here: {c}"}

    world = wh.to_dict()
    server = DashboardServer(lambda: world, lambda: mon.snapshot(now=time.monotonic()), command,
                             host=args.host, port=args.port,
                             info={"source": "edge", "controls": ["add_tasks", "block", "fail"]})
    await server.start()
    print(f"dashboard on http://localhost:{args.port}", file=sys.stderr)
    while True:
        for m in tx.poll():
            mon.ingest(m, now=time.monotonic())
        await asyncio.sleep(0.02)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", default="robot1")
    ap.add_argument("--start", nargs=2, type=int, default=[3, 6], metavar=("X", "Y"), help="start cell")
    ap.add_argument("--transport", choices=["udp", "zenoh"], default="udp")
    ap.add_argument("--connect", nargs="*", default=None, help="zenoh endpoints, e.g. tcp/192.168.1.10:7447")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--wms", action="store_true", help="run a task-source peer instead of a robot")
    ap.add_argument("--tasks", type=int, default=12)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--dashboard", action="store_true", help="run a passive dashboard peer")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args(argv)
    coro = run_dashboard(args) if args.dashboard else run_wms(args) if args.wms else run_robot(args)
    try:
        asyncio.run(coro)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
