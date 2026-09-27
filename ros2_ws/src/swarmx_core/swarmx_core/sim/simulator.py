"""Headless multi-robot warehouse simulator.

Deliberately simple physics (holonomic discs with acceleration limits) so
hundreds of runs finish in minutes; the *coordination code is exactly the code
that runs onboard* (FleetAgent over a Transport). Gazebo Harmonic is used for
the high-fidelity demo, ARGoS for large-scale runs.

Ground truth that agents never see: collision accounting, true obstacle set.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from .. import protocol
from ..agent import FAILED, AgentConfig, FleetAgent
from ..transport import InProcBus
from ..warehouse import Cell, Warehouse


@dataclass
class SimConfig:
    n_robots: int = 5
    n_tasks: int = 30
    seed: int = 0
    motion: str = "swarmx"
    allocator: str = "cbba"
    dt: float = 0.05
    max_time: float = 900.0
    comm_range: float = 20.0
    latency: float = 0.02
    jitter: float = 0.02
    loss: float = 0.02
    sensor_range: float = 2.5
    ray_range: float = 10.0              # m, line-of-sight along the aisles
    max_accel: float = 1.5
    battery_min: float = 0.85
    battery_per_m: float = 0.0015
    battery_idle_per_s: float = 0.00005
    charge_per_s: float = 0.02
    task_stream: float = 0.0             # tasks/second after the initial batch (0 = batch only)
    endless: bool = False                # keep generating tasks (dashboard demo)
    events: List[Tuple[float, str, object]] = field(default_factory=list)
    agent_overrides: Dict[str, object] = field(default_factory=dict)


@dataclass
class Robot:
    id: str
    x: float
    y: float
    th: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    battery: float = 1.0
    agent: Optional[FleetAgent] = None


class TaskSource:
    """Warehouse-management-system peer: injects tasks into the swarm.

    It is *not* a planner - it only publishes what needs moving and re-announces
    tasks until someone reports them delivered.
    """

    def __init__(self, bus: InProcBus, wh: Warehouse, rng: random.Random):
        self.ep = bus.endpoint("wms", infinite_range=True)
        self.wh = wh
        self.rng = rng
        self.tasks: Dict[str, dict] = {}
        self.done: Dict[str, float] = {}
        self.done_by: Dict[str, str] = {}
        self._n = 0
        self._last_announce = -1e9

    def new_task(self, t: float, pickup: Optional[Cell] = None, dropoff: Optional[Cell] = None) -> dict:
        self._n += 1
        tid = f"T{self._n:03d}"
        task = {"id": tid, "pickup": list(pickup or self.rng.choice(self.wh.pickups)),
                "dropoff": list(dropoff or self.rng.choice(self.wh.dropoffs)),
                "created": round(t, 2), "priority": 1.0}
        self.tasks[tid] = task
        self.ep.publish(protocol.make(protocol.TASK, "wms", t, tasks=[task]))
        return task

    def step(self, t: float) -> None:
        for msg in self.ep.poll():
            if msg["type"] == protocol.TASK_DONE and msg["task"] not in self.done:
                self.done[msg["task"]] = t
                self.done_by[msg["task"]] = msg["src"]
            elif msg["type"] == protocol.CBBA:
                for tid in msg.get("done", ()):
                    self.done.setdefault(tid, t)
        if t - self._last_announce > 2.0:
            self._last_announce = t
            pending = [task for tid, task in self.tasks.items() if tid not in self.done]
            if pending:
                self.ep.publish(protocol.make(protocol.TASK, "wms", t, tasks=pending))

    @property
    def pending(self) -> int:
        return len(self.tasks) - len(self.done)


class Simulation:
    def __init__(self, cfg: SimConfig, warehouse: Optional[Warehouse] = None):
        self.cfg = cfg
        self.wh = warehouse or Warehouse()
        self.rng = random.Random(cfg.seed)
        self.t = 0.0
        self.bus = InProcBus(cfg.comm_range, cfg.latency, cfg.jitter, cfg.loss, seed=cfg.seed)
        self.robots: Dict[str, Robot] = {}
        self.obstacles: Set[Cell] = set()
        self.failed: Set[str] = set()
        starts = self.wh.depot
        for i in range(cfg.n_robots):
            rid = f"robot{i + 1}"
            c = starts[i % len(starts)]
            x, y = self.wh.center(c)
            rb = Robot(rid, x, y, battery=self.rng.uniform(cfg.battery_min, 1.0))
            acfg = AgentConfig(motion=cfg.motion, allocator=cfg.allocator, seed=cfg.seed)
            for k, v in cfg.agent_overrides.items():
                setattr(acfg, k, v)
            rb.agent = FleetAgent(rid, self.wh, self.bus.endpoint(rid), acfg)
            rb.agent.pos = (x, y)
            self.robots[rid] = rb
        self.bus.position_of = self._pos_of
        self.wms = TaskSource(self.bus, self.wh, random.Random(cfg.seed + 1))
        for _ in range(cfg.n_tasks):
            self.wms.new_task(0.0)
        self.events = sorted(cfg.events, key=lambda e: e[0])
        self._next_stream = 1.0 / cfg.task_stream if cfg.task_stream > 0 else math.inf
        # metrics
        self.collisions = 0
        self.wall_contacts = 0
        self.zone_violations = 0     # episodes of a robot inside an aisle it holds no lock for
        self.zone_conflicts = 0      # episodes of incompatible robots inside the same aisle
        self._in_violation: Set[str] = set()
        self._in_conflict: Set[Tuple[str, str, str]] = set()
        self.min_sep = math.inf
        self._in_contact: Set[Tuple[str, str]] = set()
        self.log: List[Tuple[float, str]] = []
        self.paused = False

    def _pos_of(self, nid: str):
        r = self.robots.get(nid)
        return (r.x, r.y) if r else None

    # ------------------------------------------------------------ control
    def block(self, cells) -> None:
        self.obstacles |= {tuple(c) for c in cells if self.wh.is_free(tuple(c))}
        self.log.append((self.t, f"obstacle placed at {sorted(tuple(c) for c in cells)}"))

    def unblock(self, cells) -> None:
        self.obstacles -= {tuple(c) for c in cells}
        self.log.append((self.t, f"obstacle removed at {sorted(tuple(c) for c in cells)}"))

    def toggle_block(self, cell: Cell) -> None:
        (self.unblock if cell in self.obstacles else self.block)([cell])

    def fail(self, rid: str) -> None:
        if rid in self.robots and rid not in self.failed:
            self.failed.add(rid)
            self.bus.down.add(rid)  # radio silent too
            self.robots[rid].agent.set_failed(True)
            self.log.append((self.t, f"{rid} failure injected"))

    def recover(self, rid: str) -> None:
        if rid in self.failed:
            self.failed.discard(rid)
            self.bus.down.discard(rid)
            self.robots[rid].agent.set_failed(False)
            self.log.append((self.t, f"{rid} recovered"))

    def _apply_event(self, kind: str, arg) -> None:
        if kind == "block":
            self.block(arg)
        elif kind == "unblock":
            self.unblock(arg)
        elif kind == "fail":
            self.fail(arg)
        elif kind == "recover":
            self.recover(arg)
        elif kind == "tasks":
            for _ in range(int(arg)):
                self.wms.new_task(self.t)

    # --------------------------------------------------------------- step
    def _body_cells(self, x: float, y: float, r: float = 0.32) -> Set[Cell]:
        out = set()
        for cx in range(math.floor(x - r), math.floor(x + r) + 1):
            for cy in range(math.floor(y - r), math.floor(y + r) + 1):
                qx, qy = min(max(x, cx), cx + 1.0), min(max(y, cy), cy + 1.0)
                if (qx - x) ** 2 + (qy - y) ** 2 < r * r:
                    out.add((cx, cy))
        return out

    def _sense(self, rb: Robot) -> Tuple[Set[Cell], Set[Cell]]:
        """Lidar model: everything within ``sensor_range`` plus line-of-sight rays along the
        four axes (a robot at an aisle mouth sees down the whole aisle)."""
        rng = self.cfg.sensor_range
        occupied = set(self.obstacles)
        for f in self.failed:  # a dead robot is just an obstacle to everybody else's lidar
            occupied |= self._body_cells(self.robots[f].x, self.robots[f].y, r=0.22)
        blocked, free = set(), set()
        cx, cy = int(rb.x), int(rb.y)
        r = int(rng) + 1
        for y in range(cy - r, cy + r + 1):
            for x in range(cx - r, cx + r + 1):
                c = (x, y)
                if not self.wh.is_free(c):
                    continue
                if math.hypot(x + 0.5 - rb.x, y + 0.5 - rb.y) > rng:
                    continue
                (blocked if c in occupied else free).add(c)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            for k in range(1, int(self.cfg.ray_range) + 1):
                c = (cx + dx * k, cy + dy * k)
                if not self.wh.is_free(c):
                    break
                if c in occupied:
                    blocked.add(c)
                    break
                free.add(c)
        return blocked, free

    def step(self) -> None:
        cfg = self.cfg
        dt = cfg.dt
        self.t = round(self.t + dt, 6)
        t = self.t
        while self.events and self.events[0][0] <= t:
            _, kind, arg = self.events.pop(0)
            self._apply_event(kind, arg)
        if t >= self._next_stream:
            self.wms.new_task(t)
            self._next_stream += 1.0 / cfg.task_stream
        if cfg.endless and self.wms.pending < max(2 * cfg.n_robots, 6):
            self.wms.new_task(t)
        self.bus.advance(t)
        self.wms.step(t)

        for rb in self.robots.values():
            ag = rb.agent
            if rb.id not in self.failed:
                seen = [(o.x, o.y) for o in self.robots.values()
                        if o is not rb and math.hypot(o.x - rb.x, o.y - rb.y) <= self.cfg.sensor_range]
                ag.observe(*self._sense(rb), robots=seen)
            cmd = ag.step(t, dt, rb.x, rb.y, rb.th, rb.vx, rb.vy, rb.battery)
            if rb.id in self.failed:
                cmd = (0.0, 0.0)
            dvx, dvy = cmd[0] - rb.vx, cmd[1] - rb.vy
            dv = math.hypot(dvx, dvy)
            amax = cfg.max_accel * dt
            if dv > amax:
                dvx, dvy = dvx * amax / dv, dvy * amax / dv
            rb.vx += dvx
            rb.vy += dvy
            sp = math.hypot(rb.vx, rb.vy)
            vmax = ag.cfg.max_speed
            if sp > vmax:
                rb.vx, rb.vy = rb.vx * vmax / sp, rb.vy * vmax / sp
                sp = vmax
            rb.x += rb.vx * dt
            rb.y += rb.vy * dt
            if sp > 0.05:
                rb.th = math.atan2(rb.vy, rb.vx)
            if ag.status == "charging":
                rb.battery = min(1.0, rb.battery + cfg.charge_per_s * dt)
            else:
                rb.battery = max(0.0, rb.battery - cfg.battery_per_m * sp * dt - cfg.battery_idle_per_s * dt)
            for ev in ag.events:
                self.log.append((t, ev))
            ag.events.clear()
        self._account()

    def _account(self) -> None:
        rs = list(self.robots.values())
        radius = rs[0].agent.cfg.radius if rs else 0.3
        contact = set()
        for i in range(len(rs)):
            a = rs[i]
            for j in range(i + 1, len(rs)):
                b = rs[j]
                d = math.hypot(a.x - b.x, a.y - b.y)
                if d < self.min_sep:
                    self.min_sep = d
                if d < 2 * radius - 1e-3:
                    contact.add((a.id, b.id))
        new = contact - self._in_contact
        if new:
            self.collisions += len(new)
            for pair in new:
                self.log.append((self.t, f"COLLISION {pair[0]} <-> {pair[1]}"))
        self._in_contact = contact
        for rb in rs:
            for (qx, qy) in self.wh.nearby_wall_points(rb.x, rb.y, radius - 0.02, self.obstacles):
                self.wall_contacts += 1
                break
        self._account_zones(rs)

    def _account_zones(self, rs) -> None:
        """Ground-truth check of the zone-lock invariants (never visible to the robots)."""
        from ..zones import lane_safe
        violating, occupants = set(), {}
        for rb in rs:
            if rb.id in self.failed or rb.agent.cfg.ghost:
                continue
            z = self.wh.zone_of(self.wh.cell_of(rb.x, rb.y))
            if z is None:
                continue
            claim = rb.agent.zones.mine.get(z)
            if claim is None or claim.state != "hold":
                violating.add(rb.id)
            else:
                occupants.setdefault(z, []).append((rb.id, claim))
        new = violating - self._in_violation
        self.zone_violations += len(new)
        for rid in new:
            self.log.append((self.t, f"ZONE VIOLATION {rid} inside an aisle without a lock"))
        self._in_violation = violating
        conflicts = set()
        for z, occ in occupants.items():
            for i in range(len(occ)):
                for j in range(i + 1, len(occ)):
                    (a, ca), (b, cb) = occ[i], occ[j]
                    if not lane_safe(ca, cb):
                        conflicts.add((z, a, b))
        newc = conflicts - self._in_conflict
        self.zone_conflicts += len(newc)
        for z, a, b in newc:
            self.log.append((self.t, f"ZONE CONFLICT {a} and {b} in {z}"))
        self._in_conflict = conflicts

    # ---------------------------------------------------------------- run
    def done(self) -> bool:
        return not self.cfg.endless and self.wms.pending == 0 and not self.events and self.cfg.task_stream <= 0

    def run(self) -> dict:
        while not self.done() and self.t < self.cfg.max_time:
            self.step()
        return self.results()

    def results(self) -> dict:
        w = self.wms
        comp = [w.done[tid] - w.tasks[tid]["created"] for tid in w.done if tid in w.tasks]
        agents = [r.agent for r in self.robots.values()]
        s = self.bus.stats
        return {
            "motion": self.cfg.motion, "allocator": self.cfg.allocator,
            "robots": self.cfg.n_robots, "tasks": len(w.tasks), "seed": self.cfg.seed,
            "completed": len(w.done), "all_done": w.pending == 0,
            "makespan": round(max(w.done.values()) if w.done and w.pending == 0 else self.t, 2),
            "mean_completion": round(sum(comp) / len(comp), 2) if comp else None,
            "collisions": self.collisions, "wall_contacts": self.wall_contacts,
            "zone_violations": self.zone_violations, "zone_conflicts": self.zone_conflicts,
            "min_separation": round(self.min_sep, 3) if self.min_sep < math.inf else None,
            "distance": round(sum(a.stats.distance for a in agents), 1),
            "zone_wait": round(sum(a.stats.zone_wait_time for a in agents), 1),
            "stop_time": round(sum(a.stats.stop_time for a in agents), 1),
            "reroutes": sum(a.stats.reroutes for a in agents),
            "deadlocks_resolved": sum(a.stats.deadlocks_resolved for a in agents),
            "estops": sum(a.stats.estops for a in agents),
            "msgs": s["sent"], "kbytes": round(s["bytes"] / 1024, 1), "dropped": s["dropped"],
            "sim_time": round(self.t, 2),
        }
