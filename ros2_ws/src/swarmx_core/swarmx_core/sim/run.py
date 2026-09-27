"""Run the lightweight simulator with the live fleet dashboard.

    python -m swarmx_core.sim.run                         # 5 robots, SwarmX, http://localhost:8080
    python -m swarmx_core.sim.run --robots 8 --scenario crossing --method stopwait
    python -m swarmx_core.sim.run --headless --duration 300   # no UI, print a summary

The dashboard is attached to the simulated radio as an ordinary *passive
peer* (infinite range): everything it shows comes from the robots' own P2P
broadcasts. Ground-truth values that no robot knows (collisions, closest
pass) are added from the simulator and labelled as such.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time

from .. import protocol
from ..dashboard.monitor import FleetMonitor
from ..dashboard.server import DashboardServer
from .scenarios import METHODS, SCENARIOS, build


class LiveSim:
    def __init__(self, scenario: str, robots: int, method: str, seed: int, endless: bool, speed: float):
        self.scenario, self.robots, self.method, self.seed = scenario, robots, method, seed
        self.endless = endless
        self.speed = speed
        self.paused = False
        self._reset(scenario, robots, method)

    def _reset(self, scenario: str, robots: int, method: str) -> None:
        self.scenario, self.robots, self.method = scenario, robots, method
        self.sim = build(scenario, robots, self.seed, method, max_time=1e9)
        self.sim.cfg.endless = self.endless
        self.monitor = FleetMonitor()
        self.ep = self.sim.bus.endpoint("dashboard", infinite_range=True)
        self.monitor.note(f"Started {scenario.replace('_', ' ')} with {robots} robots ({method})", now=0.0)

    # ------------------------------------------------------------ stepping
    def step(self, n: int) -> None:
        for _ in range(n):
            self.sim.step()
            for msg in self.ep.poll():
                self.monitor.ingest(msg, now=self.sim.t)
        # surface ground-truth events (failures, obstacles, collisions) once
        new = self.sim.log[getattr(self, "_log_seen", 0):]
        self._log_seen = len(self.sim.log)
        for t, text in new:
            if text.startswith("COLLISION"):
                self.monitor.note(f"Ground truth: {text.lower()}", kind="critical", now=t)
            elif "failure injected" in text or "recovered" in text or "obstacle" in text:
                self.monitor.note(text, kind="alert", now=t)

    # ------------------------------------------------------------ dashboard
    def world(self) -> dict:
        w = self.sim.wh.to_dict()
        w["pickups"] = self.sim.wh.pickups
        return w

    def info(self) -> dict:
        return {"source": "sim", "controls": ["pause", "speed", "add_tasks", "block", "fail", "reset"],
                "scenarios": sorted(SCENARIOS), "scenario": self.scenario, "method": self.method, "robots": self.robots}

    def snapshot(self) -> dict:
        snap = self.monitor.snapshot(now=self.sim.t)
        snap["sim"] = {"paused": self.paused, "speed": self.speed, "t": round(self.sim.t, 2),
                       "collisions": self.sim.collisions,
                       "min_separation": round(self.sim.min_sep, 3) if self.sim.min_sep < 1e9 else None,
                       "tasks_total": len(self.sim.wms.tasks)}
        return snap

    def command(self, cmd: dict) -> dict:
        c = cmd.get("cmd")
        if c == "pause":
            self.paused = True
        elif c == "resume":
            self.paused = False
        elif c == "speed":
            self.speed = max(0.1, min(float(cmd.get("value", 1.0)), 20.0))
        elif c == "add_tasks":
            for _ in range(int(cmd.get("n", 5))):
                self.sim.wms.new_task(self.sim.t)
            self.monitor.note(f"Operator added {int(cmd.get('n', 5))} tasks", now=self.sim.t)
        elif c == "toggle_block":
            cell = tuple(cmd["cell"])
            if not self.sim.wh.is_free(cell):
                return {"ok": False, "error": "not a floor cell"}
            self.sim.toggle_block(cell)
        elif c == "fail":
            self.sim.fail(cmd["robot"])
        elif c == "recover":
            self.sim.recover(cmd["robot"])
        elif c == "reset":
            method = cmd.get("method", self.method)
            scenario = cmd.get("scenario", self.scenario)
            if method not in METHODS or scenario not in SCENARIOS:
                return {"ok": False, "error": "unknown scenario or method"}
            self._reset(scenario, int(cmd.get("robots", self.robots)), method)
            return {"ok": True, "reset": True}
        else:
            return {"ok": False, "error": f"unknown command {c!r}"}
        return {"ok": True}


async def run_live(live: LiveSim, host: str, port: int) -> None:
    server = DashboardServer(live.world, live.snapshot, live.command, host=host, port=port, info=live.info())
    # the reset command can change robots/scenario: keep info in sync
    orig = live.command

    def command(cmd):
        res = orig(cmd)
        server.info = live.info()
        if res.get("reset"):
            loop.create_task(_push_world(server, live))
        return res
    server.on_command = command
    loop = asyncio.get_running_loop()
    await server.start()
    print(f"SwarmX dashboard: http://localhost:{port}  (Ctrl+C to stop)", file=sys.stderr)
    dt = live.sim.cfg.dt
    last = time.perf_counter()
    budget = 0.0
    while True:
        await asyncio.sleep(0.02)
        now = time.perf_counter()
        elapsed, last = now - last, now
        if live.paused:
            continue
        budget += elapsed * live.speed
        n = int(budget / dt)
        if n:
            budget -= n * dt
            live.step(min(n, 400))


async def _push_world(server: DashboardServer, live: LiveSim) -> None:
    text = json.dumps({"type": "world", **live.world(), "info": live.info()})
    for ws in list(server.clients):
        await ws.send_text(text)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="random", choices=sorted(SCENARIOS))
    ap.add_argument("--robots", type=int, default=5)
    ap.add_argument("--method", default="swarmx", choices=sorted(METHODS))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--speed", type=float, default=1.0, help="simulation speed factor (live mode)")
    ap.add_argument("--no-endless", action="store_true", help="stop generating tasks after the scenario batch")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--headless", action="store_true", help="run as fast as possible without the dashboard")
    ap.add_argument("--duration", type=float, default=300.0, help="headless: simulated seconds")
    args = ap.parse_args(argv)

    live = LiveSim(args.scenario, args.robots, args.method, args.seed, endless=not args.no_endless, speed=args.speed)
    if args.headless:
        steps = int(args.duration / live.sim.cfg.dt)
        live.step(steps)
        snap = live.snapshot()
        print(json.dumps({"metrics": snap["metrics"], "sim": snap["sim"]}, indent=2))
        return 0
    try:
        asyncio.run(run_live(live, args.host, args.port))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
