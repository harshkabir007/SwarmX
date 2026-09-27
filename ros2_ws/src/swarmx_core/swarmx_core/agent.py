"""FleetAgent - the per-robot decentralized coordination brain.

One instance runs onboard each robot (Raspberry Pi / Jetson). It is pure
Python, transport-agnostic and simulator-agnostic: the host (lightweight sim,
ROS 2 node, or bare-metal runtime) calls :meth:`FleetAgent.step` with the
robot's pose and gets a holonomic velocity command back.

Layers (bottom-up):
  1. P2P state sharing (pose, velocity, intent, zone claims) via a Transport
  2. Task allocation - CBBA consensus auction ("cbba") or greedy nearest-task
     claiming ("greedy", the traditional approach, used as a baseline)
  3. Route planning on the warehouse graph, congestion-aware, with live
     re-routing around busy or blocked aisles
  4. Choke-point coordination - decentralized zone locks with look-ahead
  5. Local collision avoidance - ORCA ("swarmx") or the traditional
     stop-and-wait rule ("stopwait", baseline)
"""
from __future__ import annotations

import math
import random
import zlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

from . import orca, protocol
from .cbba import CBBAAgent
from .planner import GridPlanner, path_length, zone_traversals
from .transport import Transport
from .warehouse import Cell, Warehouse
from .zones import ZoneLocks

IDLE, PARKING = "idle", "parking"
TO_PICKUP, PICKING = "to_pickup", "picking"
TO_DROPOFF, DROPPING = "to_dropoff", "dropping"
TO_CHARGER, CHARGING = "to_charger", "charging"
FAILED = "failed"

STATIONARY = {PICKING, DROPPING, CHARGING, FAILED}
CARRYING = {PICKING, TO_DROPOFF, DROPPING}


@dataclass
class AgentConfig:
    motion: str = "swarmx"          # "swarmx" (lookahead zones + ORCA + reroute) | "stopwait"
    allocator: str = "cbba"         # "cbba" | "greedy"
    radius: float = 0.3
    safety_margin: float = 0.05
    max_speed: float = 1.0
    decel: float = 1.0
    state_period: float = 0.1
    cbba_period: float = 0.5
    neighbor_dist: float = 4.0
    wall_dist: float = 1.2
    orca_horizon: float = 2.0
    orca_horizon_obst: float = 0.4
    zone_lookahead: float = 7.0     # metres along the route at which zones are claimed
    zone_commit: float = 1.6        # metres before entry at which a granted claim becomes a hold
    settle: float = 0.3
    pick_time: float = 3.0
    drop_time: float = 2.0
    battery_low: float = 0.2
    battery_full: float = 0.95
    peer_timeout: float = 1.0
    stuck_time: float = 6.0
    reroute: bool = True
    congestion_routing: bool = True
    max_bundle: int = 3
    diagonal: bool = True           # allow 8-connected routes (swarmx); baseline is always 4-connected
    ghost: bool = False             # benchmark lower bound: robots ignore each other entirely
    intent_margin: float = 0.5      # s of slack added around each announced zone traversal
    intent_horizon: float = 1e9     # s; conflicts further ahead are discounted exp(-dt/horizon)
    intent_weight: float = 2.0      # metres of penalty per second of predicted conflict (x speed)
    static_load_penalty: bool = False  # legacy time-agnostic penalty (superseded by intents)
    block_ttl: float = 60.0         # s; obstacle reports expire unless someone re-confirms them
    block_refresh: float = 10.0     # s; re-broadcast a still-visible obstacle at most this often
    stop_dist: float = 1.5          # stop-and-wait: protective field (>= braking distance at 2 m/s closing)
    seed: int = 0


@dataclass
class Peer:
    msg: dict
    t_rx: float


@dataclass
class AgentStats:
    tasks_done: int = 0
    distance: float = 0.0
    zone_wait_time: float = 0.0
    stop_time: float = 0.0
    reroutes: int = 0
    deadlocks_resolved: int = 0
    estops: int = 0
    msgs_sent: int = 0


class FleetAgent:
    def __init__(self, robot_id: str, warehouse: Warehouse, transport: Transport,
                 config: Optional[AgentConfig] = None):
        self.id = robot_id
        self.wh = warehouse
        self.tx = transport
        self.cfg = config or AgentConfig()
        self.rng = random.Random(zlib.crc32(f"{robot_id}:{self.cfg.seed}".encode()))
        self.planner = GridPlanner(warehouse)
        if self.cfg.motion == "stopwait":
            self.planner.lanes = warehouse.lanes()   # traditional one-way lanes
            self.planner.allow_diagonal = False       # orthogonal grid moves (block control)
        if not self.cfg.diagonal:
            self.planner.allow_diagonal = False
        self.zones = ZoneLocks(robot_id, settle=self.cfg.settle,
                               zone_len={z.zid: len(z.cells) for z in warehouse.zones.values()})
        # baseline traffic control: every grid cell is a lock (block-based AGV control)
        self.cells = ZoneLocks(robot_id, settle=0.15, stale_claim=1.0, stale_hold=5.0)
        self.cbba = CBBAAgent(robot_id, self._travel_time, service_time=0.5 * (self.cfg.pick_time + self.cfg.drop_time),
                              max_bundle=self.cfg.max_bundle)
        self.peers: Dict[str, Peer] = {}

        # physical state (fed by host)
        self.t = 0.0
        self.pos: Tuple[float, float] = (0.0, 0.0)
        self.vel: Tuple[float, float] = (0.0, 0.0)
        self.theta = 0.0
        self.battery = 1.0

        # task / route state
        self.status = IDLE
        self.task: Optional[str] = None
        self.task_claim_t = 0.0          # greedy allocator claim time
        self.task_locked = False
        self.route: List[Cell] = []
        self.route_idx = 0
        self.goal_idx = 0                # index of the current service point in route
        self.goal_kind = ""
        self.pickup_idx: Optional[int] = None
        self.traversals: List[dict] = []
        self.service_until = 0.0
        self.park_cell: Optional[Cell] = None
        self.charger: Optional[Cell] = None

        # coordination state
        self.waiting_zone: Optional[str] = None
        self.wait_since = 0.0
        self.rerouted_for: Optional[str] = None
        self.blocked_by: Optional[str] = None
        self.blocked_since = 0.0
        self._progress = (0.0, math.inf)
        self._jitter_until = 0.0
        self._jitter = (0.0, 0.0)
        self._cand = (None, 0.0)
        self._last_state_tx = -1e9
        self._last_cbba_tx = -1e9
        self._cbba_dirty = True
        self._need_replan = False
        self._evade_deadline = 0.0
        self._evade_dwell = 0.0
        self._next_resolve = 0.0
        self._intruding = False
        self._sensed_robots: List[Tuple[float, float]] = []
        self._mouths: Set[Cell] = {c for z in warehouse.zones.values() for c in z.ends.values()}
        self._wait_spot: Optional[Cell] = None
        self._wait_key: Optional[tuple] = None
        self._dock_since: Optional[float] = None
        self.last_limit = 0
        self._blocked_seen: Dict[Cell, float] = {}   # cell -> last confirmation time (own or peer)
        self._block_tx: Dict[Cell, float] = {}
        self._next_expiry = 0.0
        self.stats = AgentStats()
        self.events: List[str] = []
        self._ev_scan = 0
        self._ev_outbox: List[str] = []

    # =============================================================== host API
    def step(self, t: float, dt: float, x: float, y: float, theta: float,
             vx: float, vy: float, battery: float) -> Tuple[float, float]:
        self.stats.distance += math.hypot(x - self.pos[0], y - self.pos[1]) if self.t > 0 else 0.0
        self.t, self.pos, self.theta, self.vel, self.battery = t, (x, y), theta, (vx, vy), battery
        self._receive(t)
        if self.status == FAILED:
            self._broadcast(t)
            return (0.0, 0.0)
        self._prune_peers(t)
        self._expire_obstacles(t)
        self._allocate(t)
        self._fsm(t)
        if self._need_replan:
            self._need_replan = False
            self._replan(t)
        cmd = self._motion(t, dt)
        self._collect_notable()
        self._broadcast(t)
        return cmd

    def observe(self, blocked: Set[Cell], free: Set[Cell], robots: Optional[Sequence[Tuple[float, float]]] = None) -> None:
        """Perception input (lidar/depth): cells seen occupied / free within sensor range, and
        robot-shaped obstacles detected nearby (``robots``, world positions). Detected robots
        that do not match a live P2P peer are avoided as non-cooperative obstacles."""
        if robots is not None:
            self._sensed_robots = list(robots)
        now = self.t
        new_b, refresh = [], []
        mine = self._body_cells()
        for c in blocked:
            if not self.wh.is_free(c) or c in mine:
                continue  # never believe an obstacle where my own body is
            self._blocked_seen[c] = now
            if c not in self.planner.blocked:
                new_b.append(c)
            elif now - self._block_tx.get(c, -1e9) > self.cfg.block_refresh:
                refresh.append(c)  # keep peers' copies alive while we can still see it
        new_f = [c for c in free if c in self.planner.blocked]
        for c in new_f:
            self._blocked_seen.pop(c, None)
        if new_b or new_f or refresh:
            for c in new_b + refresh:
                self._block_tx[c] = now
            self._publish(protocol.make(protocol.BLOCKED, self.id, now, cells=[list(c) for c in new_b + refresh],
                                        cleared=[list(c) for c in new_f]))
        if new_b or new_f:
            self.planner.set_blocked(new_b, True)
            self.planner.set_blocked(new_f, False)
            self._world_changed()

    def _expire_obstacles(self, t: float) -> None:
        """Forget obstacle reports nobody has confirmed for ``block_ttl`` (costmap-style decay)."""
        if t < self._next_expiry:
            return
        self._next_expiry = t + 1.0
        old = [c for c, ts in self._blocked_seen.items() if t - ts > self.cfg.block_ttl]
        for c in old:
            del self._blocked_seen[c]
        if old and self.planner.set_blocked(old, False):
            self.events.append(f"{self.id} forgot unconfirmed obstacle(s) {old}")
            self._world_changed()

    def set_failed(self, failed: bool) -> None:
        if failed and self.status != FAILED:
            self.status = FAILED
            self.cbba.release_all(keep_locked=False)  # if the radio still works, peers re-bid at once
            self.cbba.enabled = False
            self._cbba_dirty = True
            self.task, self.task_locked = None, False
            self.events.append(f"{self.id} FAILED")
        elif not failed and self.status == FAILED:
            # reboot: forget everything we held; peers re-allocated it while we were down
            self.status = IDLE
            self.cbba.release_all(keep_locked=False)
            self.cbba.locked, self.cbba.locked_picked = None, False
            self.cbba.enabled = True
            self.cbba.changed = True
            self._cbba_dirty = True
            self.task = None
            self.task_locked = False
            self.route, self.traversals = [], []
            for z in list(self.zones.mine):
                self.zones.release(z)
            self.events.append(f"{self.id} recovered")

    # ============================================================ comms in
    def _receive(self, t: float) -> None:
        for msg in self.tx.poll():
            typ, src = msg.get("type"), msg.get("src")
            if src == self.id:
                continue
            if typ == protocol.STATE:
                self.peers[src] = Peer(msg, t)
                self.zones.update_peer(src, msg.get("zones", []), t)
                if "cells" in msg:
                    self.cells.update_peer(src, msg["cells"], t)
            elif typ == protocol.CBBA:
                if self.cfg.allocator == "cbba":
                    self.cbba.consensus(msg, t)
                else:
                    for tid in msg.get("done", ()):
                        if tid not in self.cbba.done:
                            self.cbba.mark_done(tid, t)
                    for spec in msg.get("tasks", {}).values():
                        self.cbba.add_task(spec)
            elif typ == protocol.TASK:
                for spec in msg.get("tasks", [msg.get("task")]):
                    if spec:
                        self.cbba.add_task(spec)
            elif typ == protocol.TASK_DONE:
                if msg["task"] not in self.cbba.done:
                    self.cbba.mark_done(msg["task"], t)
            elif typ == protocol.BLOCKED:
                mine = self._body_cells()
                cells = [tuple(c) for c in msg.get("cells", []) if tuple(c) not in mine]
                cleared = [tuple(c) for c in msg.get("cleared", [])]
                ts = msg.get("t", t)
                for c in cells:
                    self._blocked_seen[c] = max(self._blocked_seen.get(c, -1e9), ts)
                for c in cleared:
                    self._blocked_seen.pop(c, None)
                ch = self.planner.set_blocked(cells, True)
                ch |= self.planner.set_blocked(cleared, False)
                if ch:
                    self._world_changed()
            elif typ == protocol.CMD:
                if msg.get("target") in (self.id, "*"):
                    self._handle_cmd(msg)

    def _handle_cmd(self, msg: dict) -> None:
        cmd = msg.get("cmd")
        if cmd == "fail":
            self.set_failed(True)
        elif cmd == "recover":
            self.set_failed(False)

    def _prune_peers(self, t: float) -> None:
        for rid in [r for r, p in self.peers.items() if t - p.t_rx > 30.0]:
            del self.peers[rid]
            self.zones.forget_peer(rid)
            self.cells.forget_peer(rid)

    def fresh_peers(self, t: float):
        for rid, p in self.peers.items():
            if t - p.t_rx <= self.cfg.peer_timeout:
                yield rid, p

    # ============================================================ comms out
    def _publish(self, msg: dict) -> None:
        self.tx.publish(msg)
        self.stats.msgs_sent += 1

    NOTABLE = ("rerouted", "resolved deadlock", "evades", "detours", "low battery", "no route")

    def _collect_notable(self) -> None:
        # hosts may clear self.events after each step (the simulator does); track by index
        if self._ev_scan > len(self.events):
            self._ev_scan = 0
        self._ev_outbox += [e for e in self.events[self._ev_scan:] if any(k in e for k in self.NOTABLE)]
        self._ev_scan = len(self.events)

    def _new_notable(self) -> List[str]:
        out, self._ev_outbox = self._ev_outbox[-3:], []
        return out

    def _broadcast(self, t: float) -> None:
        if t - self._last_state_tx >= self.cfg.state_period:
            self._last_state_tx = t
            s = self.stats
            self._publish(protocol.make(
                protocol.STATE, self.id, t,
                x=round(self.pos[0], 3), y=round(self.pos[1], 3), th=round(self.theta, 3),
                vx=round(self.vel[0], 3), vy=round(self.vel[1], 3), bat=round(self.battery, 3),
                st=self.status, task=self.task, task_t=round(self.task_claim_t, 3), locked=self.task_locked,
                goal=list(self.route[self.goal_idx]) if self.route else None,
                path=[list(c) for c in self.route[self.route_idx:self.route_idx + 12]],
                zones=self.zones.wire(), intent=self._intent_wire(t), wait=self.waiting_zone, blocked_by=self.blocked_by,
                **({"cells": self.cells.wire()} if self.cfg.motion == "stopwait" else {}),
                mode=self.cfg.motion, alloc=self.cfg.allocator, ev=self._new_notable(), arr=self._arriving(),
                bundle=list(self.cbba.path) if self.cfg.allocator == "cbba" else ([self.task] if self.task else []),
                stats={"done": s.tasks_done, "dist": round(s.distance, 1), "wait": round(s.zone_wait_time + s.stop_time, 1),
                       "reroutes": s.reroutes, "deadlocks": s.deadlocks_resolved}))
        period = self.cfg.cbba_period
        if (self._cbba_dirty and t - self._last_cbba_tx >= 0.1) or t - self._last_cbba_tx >= period:
            self._last_cbba_tx = t
            self._cbba_dirty = False
            fields = self.cbba.message_fields(t)
            if self.cfg.allocator != "cbba":
                fields = {"y": {}, "z": {}, "s": {}, "tasks": fields["tasks"], "done": fields["done"]}
            self._publish(protocol.make(protocol.CBBA, self.id, t, **fields))

    # ========================================================== allocation
    def _travel_time(self, a: Cell, b: Cell) -> float:
        return self.planner.distance(a, b) / (0.8 * self.cfg.max_speed)

    def _cell(self) -> Cell:
        c = self.wh.cell_of(*self.pos)
        if self.planner.passable(c):
            return c
        best, bd = c, math.inf
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (c[0] + dx, c[1] + dy)
                if self.planner.passable(n):
                    d = math.dist(self.wh.center(n), self.pos)
                    if d < bd:
                        best, bd = n, d
        return best

    def _can_take_tasks(self) -> bool:
        return self.status not in (TO_CHARGER, CHARGING, FAILED) and self.battery > self.cfg.battery_low

    def _allocate(self, t: float) -> None:
        cb = self.cbba
        if self.cfg.allocator == "cbba":
            if cb.expire_agents(t):
                self._cbba_dirty = True
            cb.enabled = self._can_take_tasks()
            if cb.changed:
                cb.changed = False
                cb.start = self._cell()
                cb.build_bundle()
                cb.changed = False
                self._cbba_dirty = True
            desired = cb.current_task()
        else:
            desired = self._greedy_choice(t)
        self._adopt_task(desired, t)

    def _greedy_choice(self, t: float) -> Optional[str]:
        """Traditional first-come nearest-task claiming (baseline allocator)."""
        cb = self.cbba
        claims: Dict[str, Tuple[float, str]] = {}
        for rid, p in self.fresh_peers(t):
            tid = p.msg.get("task")
            if tid:
                key = (-1.0 if p.msg.get("locked") else p.msg.get("task_t", 0.0), rid)
                if tid not in claims or key < claims[tid]:
                    claims[tid] = key
        if self.task and self.task in cb.tasks:
            mine = (-1.0 if self.task_locked else self.task_claim_t, self.id)
            other = claims.get(self.task)
            if other is None or mine < other:
                return self.task
        if not self._can_take_tasks() or self.status in CARRYING:
            return self.task if self.status in CARRYING else None
        here = self._cell()
        best, bd = None, math.inf
        for tid, task in cb.tasks.items():
            if tid in claims or tid in cb.skip:
                continue  # a fresh peer claim exists (and beat ours if it was ours), or unreachable
            d = self.planner.distance(here, tuple(task["pickup"]))
            if d < bd:
                best, bd = tid, d
        if best is not None and best != self.task:
            self.task_claim_t = t
        return best

    def _adopt_task(self, desired: Optional[str], t: float) -> None:
        if self.status in CARRYING or self.status in (TO_CHARGER, CHARGING):
            return
        if desired == self.task:
            self._cand = (desired, t)
            return
        if self.task_locked:
            return
        if self.status == TO_PICKUP and desired is not None and self.cfg.allocator == "cbba":
            if self._cand[0] != desired:
                self._cand = (desired, t)
                return
            if t - self._cand[1] < 0.3:  # hysteresis against transient consensus flips
                return
        old = self.task
        self.task = desired
        if desired is None:
            if self.status == TO_PICKUP:
                self.events.append(f"{self.id} lost {old}")
                self.status = IDLE
                self.route = []
            return
        task = self.cbba.tasks[desired]
        self.status = TO_PICKUP
        self.task_locked = False
        self._set_route([tuple(task["pickup"]), tuple(task["dropoff"])], ["pickup", "dropoff"], t)

    def task_locked_picked(self) -> bool:
        return self.status in CARRYING

    def _abandon_task(self, t: float, why: str) -> None:
        """Hand an unreachable (not yet picked) task back to the fleet."""
        tid = self.task
        if tid is None:
            return
        if self.cfg.allocator == "cbba":
            self.cbba.abandon(tid)
            self._cbba_dirty = True
        else:
            self.cbba.skip.add(tid)
        self.task, self.task_locked = None, False
        self.status = IDLE
        self.route, self.traversals = [], []
        self.events.append(f"{self.id} gave up {tid} ({why}); re-auctioned")

    def _lock_task(self, picked: bool) -> None:
        self.task_locked = True
        if self.cfg.allocator == "cbba" and self.task:
            self.cbba.lock(self.task, picked)
            self._cbba_dirty = True

    def _world_changed(self) -> None:
        if self.cfg.allocator == "cbba":
            self.cbba.release_all(keep_locked=True)
            self._cbba_dirty = True
        else:
            self.cbba.skip.clear()
        if self.route and any(c in self.planner.blocked for c in self.route[self.route_idx:]):
            self._need_replan = True

    # ================================================================= FSM
    def _fsm(self, t: float) -> None:
        st = self.status
        if st in (PICKING, DROPPING):
            if st == PICKING and t >= self.service_until and self.pickup_idx is not None:
                pz = self.wh.zone_of(self.route[self.pickup_idx])
                if pz and self.zones.exit_blocked_by_stacker(pz, t):
                    return  # a robot stacked behind me is still heading in: let it settle first
            if t >= self.service_until:
                if st == PICKING:
                    self._lock_task(True)
                    self.status = TO_DROPOFF
                    self.goal_idx = len(self.route) - 1
                    self.goal_kind = "dropoff"
                else:
                    self._finish_task(t)
            return
        if st == CHARGING:
            if self.battery >= self.cfg.battery_full:
                self.status = IDLE
                self.charger = None
                self.cbba.changed = True
            return
        if self.goal_kind == "evade":
            if self._arrived() or t > self._evade_deadline:
                if self._evade_dwell == 0.0:
                    self._evade_dwell = t + 1.0
                elif t >= self._evade_dwell:
                    self._evade_dwell = 0.0
                    if not self._remaining_goals()[0]:
                        self.goal_kind, self.route, self.park_cell = "", [], None  # idle: pick a parking bay
                        self.status = IDLE
                    elif not self._replan(t):
                        self._evade_deadline = t + 1.0
            return
        if self.status in (IDLE, PARKING) and self.battery <= self.cfg.battery_low:
            self._go_charge(t)
            return
        if st == IDLE and self.task is None and self.cfg.motion == "swarmx" and t >= self._next_resolve:
            if self._make_way(t):
                return
        if st == IDLE and self.task is None:
            if self.park_cell is None or math.dist(self.pos, self.wh.center(self.park_cell)) > 0.4:
                self._go_park(t)
            return
        if st == TO_PICKUP and not self.task_locked and self.route and self.pickup_idx is not None:
            if self._dist_along(self.pickup_idx) < 3.0:
                self._lock_task(False)
        if self._arrived():
            if st == TO_PICKUP:
                self.status = PICKING
                self.service_until = t + self.cfg.pick_time
            elif st == TO_DROPOFF:
                self.status = DROPPING
                self.service_until = t + self.cfg.drop_time
            elif st == TO_CHARGER:
                self.status = CHARGING
            elif st == PARKING:
                self.status = IDLE
                self.route = []

    def _finish_task(self, t: float) -> None:
        tid = self.task
        self._publish(protocol.make(protocol.TASK_DONE, self.id, t, task=tid))
        self.cbba.mark_done(tid, t)
        self._cbba_dirty = True
        self.stats.tasks_done += 1
        self.events.append(f"{self.id} delivered {tid}")
        self.task = None
        self.task_locked = False
        self.status = IDLE
        self.route = []
        self.cbba.changed = True

    def _go_park(self, t: float) -> None:
        taken = set()
        for rid, p in self.fresh_peers(t):
            if p.msg.get("st") in (PARKING, IDLE) and p.msg.get("goal"):
                if rid < self.id or p.msg.get("st") == IDLE:
                    taken.add(tuple(p.msg["goal"]))
            taken.add(self.wh.cell_of(p.msg["x"], p.msg["y"]))
        here = self._cell()
        spots = sorted((c for c in self.wh.depot if self.planner.passable(c) and c not in taken),
                       key=lambda c: self.planner.distance(here, c))
        if not spots:
            return
        if spots[0] == here and math.dist(self.pos, self.wh.center(here)) < 0.2:
            self.park_cell = here
            return
        self.park_cell = spots[0]
        self.status = PARKING
        self._set_route([spots[0]], ["park"], t)

    def _go_charge(self, t: float) -> None:
        if self.cfg.allocator == "cbba":
            self.cbba.release_all(keep_locked=False)
            self._cbba_dirty = True
        self.task = None
        taken = {tuple(p.msg["goal"]) for _, p in self.fresh_peers(t)
                 if p.msg.get("st") in (TO_CHARGER, CHARGING) and p.msg.get("goal")}
        here = self._cell()
        docks = sorted((c for c in self.wh.chargers if c not in taken),
                       key=lambda c: self.planner.distance(here, c))
        if not docks:
            return
        self.charger = docks[0]
        self.status = TO_CHARGER
        self.events.append(f"{self.id} low battery -> charger {self.charger}")
        self._set_route([self.charger], ["charger"], t)

    # ============================================================ routing
    def _zone_penalties(self, t: float) -> Dict[str, float]:
        if self.cfg.motion != "swarmx" or not self.cfg.congestion_routing or not self.cfg.static_load_penalty:
            return {}
        v = self.cfg.max_speed
        return {z: 0.7 * load * v for z, load in self.zones.peer_load(t).items() if not self.zones.holds(z)}

    def _peer_intents(self, t: float) -> Dict[str, List[Tuple[str, float, float]]]:
        out: Dict[str, List[Tuple[str, float, float]]] = {}
        for _, p in self.fresh_peers(t):
            for zid, mode, t_in, t_out in p.msg.get("intent", ()):
                if t_out >= t:
                    out.setdefault(zid, []).append((mode, t_in, t_out))
        return out

    def _intent_penalty_fn(self, t: float, t_offset: float, extra: Optional[Dict[str, float]] = None):
        """Space-time zone penalty from peers' announced traversals (prioritized MAPF, soft).

        Entering zone Z at the time A* predicts we would arrive costs the
        overlap with every *incompatible* peer traversal of Z (opposite
        direction or turning inside); same-direction pass-throughs are free
        because they can convoy.
        """
        v = 0.8 * self.cfg.max_speed
        intents = self._peer_intents(t)
        extra = extra or {}
        pick = self.cfg.pick_time
        margin, horizon, weight = self.cfg.intent_margin, self.cfg.intent_horizon, self.cfg.intent_weight

        def pen(zid: str, entry: Optional[str], metres: float, goal_inside: bool) -> float:
            base = extra.get(zid, 0.0)
            peers = intents.get(zid)
            if not peers or entry is None:
                return base
            other = "N" if entry == "S" else "S"
            mine = entry + (entry if goal_inside else other)
            t_arr = t + t_offset + metres / v
            t_dep = t_arr + len(self.wh.zones[zid].cells) / v + (pick if goal_inside else 0.0)
            overlap = 0.0
            for mode, t_in, t_out in peers:
                if mode == mine and mode in ("SN", "NS"):
                    continue  # convoy-compatible
                o = min(t_dep, t_out) - max(t_arr, t_in)
                if o > -margin:
                    overlap += (o + margin) * math.exp(-max(t_arr - t, 0.0) / horizon)
            return base + weight * overlap * v

        return pen

    def _intent_wire(self, t: float) -> List[list]:
        """My upcoming zone traversals with estimated time windows (shared intent)."""
        if self.cfg.motion != "swarmx" or not self.route:
            return []
        v = 0.8 * self.cfg.max_speed
        out = []
        for tr in self.traversals:
            if tr["end"] < self.route_idx - 1:
                continue
            extra_pick = self.cfg.pick_time if (self.status == TO_PICKUP and self.pickup_idx is not None
                                                and self.pickup_idx < tr["start"]) else 0.0
            t_in = t + self._dist_along(tr["start"]) / v + extra_pick
            t_out = t + self._dist_along(tr["end"] + 1) / v + extra_pick
            if self.status in (TO_PICKUP, PICKING) and self.pickup_idx is not None and \
                    tr["start"] <= self.pickup_idx <= tr["end"]:
                t_out += self.cfg.pick_time
            if self._current_traversal() is tr or tr["start"] <= self.route_idx:
                t_in = min(t_in, t)
            out.append([tr["zone"], tr["mode"], round(t_in, 2), round(t_out, 2)])
            if len(out) >= 4:
                break
        return out

    def _set_route(self, goals: List[Cell], kinds: List[str], t: float,
                   extra_penalty: Optional[Dict[str, float]] = None, avoid: Optional[Set[Cell]] = None) -> bool:
        start = self._cell()
        prefix: List[Cell] = []
        cur_tr = self._current_traversal()
        if cur_tr is not None and cur_tr["end"] + 1 < len(self.route) and self.zones.holds(cur_tr["zone"]) \
                and not any(c in self.planner.blocked for c in self.route[self.route_idx:cur_tr["end"] + 2]):
            # never turn around inside a single-lane zone: finish the traversal first
            # (unless the way ahead is blocked - then reversing is the only option)
            prefix = self.route[self.route_idx:cur_tr["end"] + 1]
            start = self.route[cur_tr["end"] + 1]
        pen = self._zone_penalties(t)
        use_intents = self.cfg.motion == "swarmx" and self.cfg.congestion_routing
        parked = {}
        for _, p in self.fresh_peers(t):
            if p.msg.get("st") in STATIONARY or p.msg.get("st") == IDLE:
                parked[self.wh.cell_of(p.msg["x"], p.msg["y"])] = 6.0  # costmap-style soft avoidance
        if extra_penalty:
            for z, p in extra_penalty.items():
                pen[z] = pen.get(z, 0.0) + p
        legs = [[start]]
        cur = start
        v = 0.8 * self.cfg.max_speed
        t_offset = path_length(prefix) / v if prefix else 0.0
        for g, kind in zip(goals, kinds):
            zp = self._intent_penalty_fn(t, t_offset, pen) if use_intents else pen
            leg = self.planner.astar(cur, g, zone_penalty=zp, cell_cost=parked, avoid=avoid)
            if leg is None and avoid:
                leg = self.planner.astar(cur, g, zone_penalty=zp, cell_cost=parked)
            if leg is None:
                self.events.append(f"{self.id} no route to {g}")
                if self.task and not self.task_locked_picked() and kind in ("pickup", "dropoff") and avoid is None:
                    self._abandon_task(t, f"no route to {g}")
                return False
            legs.append(leg[1:])
            t_offset += path_length(leg) / v + (self.cfg.pick_time if kind == "pickup" else 0.0)
            cur = g
        path = prefix + [c for leg in legs for c in leg]
        # drop an immediate duplicate when prefix ends where the plan starts
        dedup = [path[0]]
        for c in path[1:]:
            if c != dedup[-1]:
                dedup.append(c)
        old_zones = {tr["zone"] for tr in self.traversals}
        self.route = dedup
        self.route_idx = 0
        self._compute_goal_indices(goals, kinds)
        self.traversals = zone_traversals(self.wh, self.route)
        new_zones = {tr["zone"] for tr in self.traversals}
        for z in old_zones - new_zones:
            if not self.zones.holds(z):
                self.zones.cancel(z)
        self._progress = (t, math.inf)
        return True

    def _compute_goal_indices(self, goals: List[Cell], kinds: List[str]) -> None:
        self.pickup_idx = None
        idx = 0
        first_goal_idx = None
        for g, kind in zip(goals, kinds):
            while idx < len(self.route) and self.route[idx] != g:
                idx += 1
            if kind == "pickup":
                self.pickup_idx = idx
            if first_goal_idx is None:
                first_goal_idx = (idx, kind)
        if self.status == TO_DROPOFF or not first_goal_idx:
            self.goal_idx, self.goal_kind = len(self.route) - 1, kinds[-1] if kinds else ""
        else:
            self.goal_idx, self.goal_kind = min(first_goal_idx[0], len(self.route) - 1), first_goal_idx[1]

    def _remaining_goals(self) -> Tuple[List[Cell], List[str]]:
        if not self.task or self.task not in self.cbba.tasks:
            if self.status == TO_CHARGER and self.charger:
                return [self.charger], ["charger"]
            if self.status == PARKING and self.park_cell:
                return [self.park_cell], ["park"]
            return [], []
        task = self.cbba.tasks[self.task]
        if self.status in (TO_DROPOFF, DROPPING, PICKING):
            return [tuple(task["dropoff"])], ["dropoff"]
        return [tuple(task["pickup"]), tuple(task["dropoff"])], ["pickup", "dropoff"]

    def _replan(self, t: float, extra_penalty=None, avoid=None) -> bool:
        goals, kinds = self._remaining_goals()
        if not goals or self.status in STATIONARY:
            return False
        return self._set_route(goals, kinds, t, extra_penalty, avoid)

    def _current_traversal(self) -> Optional[dict]:
        for tr in self.traversals:
            if tr["start"] <= self.route_idx <= tr["end"] + 1 and self.wh.zone_of(self.wh.cell_of(*self.pos)) == tr["zone"]:
                return tr
        return None

    def _dist_along(self, idx: int) -> float:
        if not self.route:
            return 0.0
        idx = max(min(idx, len(self.route) - 1), self.route_idx)
        d = math.dist(self.pos, self.wh.center(self.route[self.route_idx]))
        return d + path_length(self.route[self.route_idx:idx + 1])

    def _arriving(self) -> bool:
        """Final approach to a service point: peers give way so I can dock."""
        if self.status not in (TO_PICKUP, TO_DROPOFF, TO_CHARGER) or not self.route or self.goal_kind == "evade":
            return False
        if self.route_idx < self.goal_idx:
            return False
        return math.dist(self.pos, self.wh.center(self.route[self.goal_idx])) < 0.6

    def _arrived(self) -> bool:
        if not self.route:
            return False
        if self.route_idx < self.goal_idx:
            self._dock_since = None
            return False
        d = math.dist(self.pos, self.wh.center(self.route[self.goal_idx]))
        if d < 0.45 and self._dock_since is None:
            self._dock_since = self.t
        slow = math.hypot(*self.vel) < 0.3
        if d < 0.3 and slow:
            return True
        # jostled at a busy station: accept a slightly looser dock after a few seconds
        return d < 0.45 and slow and self._dock_since is not None and self.t - self._dock_since > 4.0

    # ======================================================= zone protocol
    def _zone_limit(self, t: float) -> int:
        """Update claims/holds; return the furthest route index we may drive to."""
        limit = self.goal_idx if self.route else 0
        self.waiting_zone = None
        if self.cfg.ghost:
            return limit
        swarm = self.cfg.motion == "swarmx"
        lookahead = self.cfg.zone_lookahead if swarm else 1.8
        here_zone = self.wh.zone_of(self.wh.cell_of(*self.pos))
        blocked_found = False
        seen: Set[str] = set()

        def behind(tr: dict) -> bool:
            return tr["end"] < self.route_idx - 1 or (tr["end"] < self.route_idx and here_zone != tr["zone"])

        upcoming = {tr["zone"] for tr in self.traversals if not behind(tr)}
        for tr in self.traversals:
            zid = tr["zone"]
            if behind(tr):
                # traversal behind us: release once our body is clear of the zone
                # (a zone we will pass again later is handled by its upcoming traversal)
                if zid not in seen and zid not in upcoming:
                    if self.zones.holds(zid) and self._clear_of_zone(zid):
                        self.zones.release(zid)
                    elif not self.zones.holds(zid):
                        self.zones.cancel(zid)
                continue
            if zid in seen:
                continue  # a route may pass the same zone twice: only the next traversal counts now
            seen.add(zid)
            inside = here_zone == zid or tr["start"] <= self.route_idx <= tr["end"]
            if swarm:
                mode, lo, hi = tr["mode"], tr["lo"], tr["hi"]
            else:
                mode, lo, hi = "XX", 0, 1 << 20   # traditional: whole aisle, exclusive
            d_entry = self._dist_along(tr["start"])
            key = t + d_entry / (0.8 * self.cfg.max_speed)  # precedence = expected entry time
            if self.zones.holds(zid) and not inside and self._clear_of_zone(zid) \
                    and d_entry > self.cfg.zone_commit + 2.0:  # well beyond any fresh grant (<= ~2.5 m)
                # invariant: holds exist only inside / right at the entry. A hold left over from
                # an earlier pass through this zone is returned so we never hold-and-wait outside.
                self.zones.release(zid)
            if inside:
                self.zones.claim(zid, mode, t, key, lo, hi)
                self.zones.hold(zid)
            if blocked_found:
                if not self.zones.holds(zid):
                    self.zones.cancel(zid)
                continue
            if not self.zones.holds(zid) and not swarm:
                wait_idx = tr["start"] - 2
                at_wait = self.route_idx > wait_idx or wait_idx < 0 or (
                    self.route_idx == wait_idx and math.dist(self.pos, self.wh.center(self.route[wait_idx])) < 0.3
                    and math.hypot(*self.vel) < 0.1)
                if not at_wait:
                    if d_entry <= lookahead + 2.0:
                        limit = min(limit, max(wait_idx, self.route_idx))  # traditional: stop, then request
                        blocked_found = True
                    continue
            if not self.zones.holds(zid):
                if d_entry <= lookahead:
                    self.zones.claim(zid, mode, t, key, lo, hi, allow_rekey=d_entry > 2.5)
                else:
                    continue
                if self.zones.can_enter(zid, t):
                    if d_entry <= self.cfg.zone_commit or self.route_idx >= tr["start"] - 1:
                        self.zones.hold(zid)
                else:
                    stop = tr["start"] - 2 if self.route_idx <= tr["start"] - 2 else tr["start"] - 1
                    limit = min(limit, max(stop, self.route_idx))
                    self.waiting_zone = zid
                    blocked_found = True
            # announce when I expect to be out of this zone (for peers' routing / waits)
            eta = (self._dist_along(tr["end"] + 1)) / max(self.cfg.max_speed * 0.8, 0.1)
            has_pick = self.pickup_idx is not None and tr["start"] <= self.pickup_idx <= tr["end"]
            if has_pick and self.status == TO_PICKUP:
                eta += self.cfg.pick_time
            phase = ("in" if self.status == TO_PICKUP else "svc" if self.status == PICKING else "out") \
                if has_pick else "out"
            zcells = self.wh.zones[zid].cells
            here = self.wh.cell_of(*self.pos)
            cur = zcells.index(here) if here in zcells else -1
            self.zones.update(zid, eta=eta, phase=phase, cur=cur)
        # drop claims on zones that are no longer on the route
        on_route = {tr["zone"] for tr in self.traversals}
        for zid in list(self.zones.mine):
            if zid not in on_route and (not self.zones.holds(zid) or self._clear_of_zone(zid)):
                self.zones.release(zid)
        return limit

    def _clear_of_zone(self, zid: str) -> bool:
        cells = self.wh.zones[zid].cells
        x0 = min(c[0] for c in cells)
        x1 = max(c[0] for c in cells) + 1
        y0 = min(c[1] for c in cells)
        y1 = max(c[1] for c in cells) + 1
        dx = max(x0 - self.pos[0], 0.0, self.pos[0] - x1)
        dy = max(y0 - self.pos[1], 0.0, self.pos[1] - y1)
        return math.hypot(dx, dy) > self.cfg.radius + 0.05

    def _maybe_reroute(self, t: float) -> None:
        """If waiting at a busy choke point costs more than a detour, take the detour."""
        z = self.waiting_zone
        if not (self.cfg.reroute and self.cfg.motion == "swarmx") or z is None:
            return
        if t - self.wait_since < 0.5 or self.rerouted_for == z:
            return
        self.rerouted_for = z
        goals, kinds = self._remaining_goals()
        if any(self.wh.zone_of(g) == z for g in goals):
            return  # the service point is inside that zone; must wait
        v = 0.8 * self.cfg.max_speed
        tr = next((x for x in self.traversals if x["zone"] == z), None)
        my_eta = self._dist_along(tr["start"]) / v if tr else 0.0
        est_wait = max(0.0, sum(max(eta, 0.5) for _, _, eta in self.zones.blockers(z, t)) - my_eta)
        stay = self._dist_along(len(self.route) - 1) / v + est_wait
        old_route, old_idx, old_tr = self.route, self.route_idx, self.traversals
        old_goal = (self.goal_idx, self.goal_kind, self.pickup_idx)
        if not self._replan(t, extra_penalty={z: 1000.0}):
            return
        new_cost = self._dist_along(len(self.route) - 1) / v
        if any(tr["zone"] == z for tr in self.traversals) or new_cost + 1.0 >= stay:
            self.route, self.route_idx, self.traversals = old_route, old_idx, old_tr
            self.goal_idx, self.goal_kind, self.pickup_idx = old_goal
            return
        self.zones.cancel(z)
        self.stats.reroutes += 1
        self.events.append(f"{self.id} rerouted around {z} (saves {stay - new_cost:.1f}s)")

    def _make_way(self, t: float) -> bool:
        """Idle robot: step aside if a busy peer's announced path runs through my cell."""
        here = self.wh.cell_of(*self.pos)
        for rid, p in self.fresh_peers(t):
            m = p.msg
            if m.get("st") in (IDLE, FAILED) or m.get("st") in STATIONARY:
                continue
            if math.hypot(m["x"] - self.pos[0], m["y"] - self.pos[1]) > 3.0:
                continue
            if any(tuple(c) == here for c in m.get("path", [])[:8]):
                self._next_resolve = t + 2.0
                return self._evade(t, rid)
        return False

    def _evade(self, t: float, other_id: Optional[str]) -> bool:
        """Step aside to a nearby free cell off ``other_id``'s announced path, then resume."""
        if self._current_traversal() is not None:
            return False  # never manoeuvre inside a single-lane zone
        occupied = {self.wh.cell_of(p.msg["x"], p.msg["y"]) for _, p in self.fresh_peers(t)}
        for _, p in self.fresh_peers(t):
            occupied |= {tuple(int(v) for v in e[0].split(",")) for e in p.msg.get("cells", []) if e[3] == "hold"}
        avoid = set(occupied)
        opos = None
        if other_id in self.peers:
            om = self.peers[other_id].msg
            avoid |= {tuple(c) for c in om.get("path", [])}
            opos = (om["x"], om["y"])
        here = self._cell()
        best, best_score = None, math.inf
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                c = (here[0] + dx, here[1] + dy)
                if c == here or c in avoid or not self.planner.passable(c) or self.wh.zone_of(c):
                    continue
                path = self.planner.astar(here, c, avoid=occupied)
                if not path or len(path) > 5:
                    continue
                score = path_length(path) - (0.7 * math.dist(self.wh.center(c), opos) if opos else 0.0)
                if score < best_score:
                    best, best_score = c, score
        if best is None:
            return False
        goals, kinds = self._remaining_goals()
        self._set_route([best], ["evade"], t)
        self.goal_idx, self.goal_kind = len(self.route) - 1, "evade"
        self._evade_deadline = t + 6.0
        self._evade_dwell = 0.0
        self.stats.deadlocks_resolved += 1
        self.events.append(f"{self.id} evades {other_id} -> {best}")
        return True

    # ============================================================== motion
    def _advance(self, limit: int) -> None:
        r = self.route
        while self.route_idx < min(limit, len(r) - 1):
            c = self.wh.center(r[self.route_idx])
            d = math.dist(self.pos, c)
            i = self.route_idx
            if 0 < i < len(r) - 1:
                a = (r[i][0] - r[i - 1][0], r[i][1] - r[i - 1][1])
                b = (r[i + 1][0] - r[i][0], r[i + 1][1] - r[i][1])
                straight = a == b
            else:
                straight = False
            turn_tol = 0.3 if self.cfg.motion == "swarmx" else 0.08
            if d < (0.6 if straight else turn_tol):
                self.route_idx += 1
            else:
                break

    def _pref_velocity(self, limit: int) -> Tuple[float, float]:
        if not self.route:
            return (0.0, 0.0)
        target = self.wh.center(self.route[min(self.route_idx, len(self.route) - 1)])
        dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        d = math.hypot(dx, dy)
        remain = self._dist_along(limit)
        if remain < 0.03 or d < 1e-6:
            return (0.0, 0.0)
        speed = min(self.cfg.max_speed, math.sqrt(2.0 * self.cfg.decel * max(remain - 0.02, 0.0)))
        # slow down for the next sharp turn (same physics for every coordination mode)
        r = self.route
        acc = d
        for i in range(self.route_idx, min(limit, len(r) - 2, self.route_idx + 4) + 1):
            if i >= 1:
                a = (r[i][0] - r[i - 1][0], r[i][1] - r[i - 1][1])
                b = (r[i + 1][0] - r[i][0], r[i + 1][1] - r[i][1])
                cosang = (a[0] * b[0] + a[1] * b[1]) / (math.hypot(*a) * math.hypot(*b))
                if cosang < 0.75:
                    v_turn = 0.35 if cosang < 0.2 else 0.6
                    speed = min(speed, math.sqrt(v_turn * v_turn + 2.0 * self.cfg.decel * max(acc - 0.05, 0.0)))
                    break
            if i + 1 < len(r):
                acc += math.dist(r[i], r[i + 1])
        return (dx / d * speed, dy / d * speed)

    def _neighbors(self, t: float) -> List[orca.Neighbor]:
        if self.cfg.ghost:
            return []
        out = []
        r = self.cfg.radius + self.cfg.safety_margin
        for rid, p in self.fresh_peers(t):
            m = p.msg
            age = min(max(t - m.get("t", t), 0.0), 0.3)
            px, py = m["x"] + m.get("vx", 0.0) * age, m["y"] + m.get("vy", 0.0) * age
            if math.hypot(px - self.pos[0], py - self.pos[1]) > self.cfg.neighbor_dist:
                continue
            resp = 1.0 if (m.get("st") in STATIONARY or m.get("mode") != "swarmx") else 0.5
            out.append(orca.Neighbor((px, py), (m.get("vx", 0.0), m.get("vy", 0.0)), r, resp))
        # sensed bodies without a live radio link (failed robot, foreign vehicle): full responsibility
        for (sx, sy) in self._sensed_robots:
            if math.hypot(sx - self.pos[0], sy - self.pos[1]) > self.cfg.neighbor_dist:
                continue
            if any(math.hypot(nb.pos[0] - sx, nb.pos[1] - sy) < 0.5 for nb in out):
                continue
            out.append(orca.Neighbor((sx, sy), (0.0, 0.0), r, 1.0))
        return out

    def _motion(self, t: float, dt: float) -> Tuple[float, float]:
        cfg = self.cfg
        if self.status in STATIONARY or not self.route:
            limit, pref = 0, (0.0, 0.0)
            self.waiting_zone = None
            self.traversals = self.traversals if self.route else []
            if self.route:
                self._zone_limit(t)
            if cfg.motion == "stopwait":
                self._reserve_cells(t, -1)
        else:
            limit = self._zone_limit(t)
            if self.waiting_zone is not None:
                if self.wait_since == 0.0:
                    self.wait_since = t
                n_before = self.stats.reroutes
                self._maybe_reroute(t)
                if self.stats.reroutes != n_before:
                    limit = self._zone_limit(t)
            else:
                self.wait_since = 0.0
                self.rerouted_for = None
            if cfg.motion == "stopwait":
                limit = self._reserve_cells(t, limit)
                if limit < self.route_idx:
                    limit = -1
            if cfg.motion == "stopwait" and self._intruding:
                # overlapping someone else's cell: back into the centre of the cell we are in
                c = self.wh.center(self.wh.cell_of(*self.pos))
                dx, dy = c[0] - self.pos[0], c[1] - self.pos[1]
                d = math.hypot(dx, dy)
                pref = (dx / d * min(0.3, d), dy / d * min(0.3, d)) if d > 0.02 else (0.0, 0.0)
            elif limit >= 0:
                self._advance(limit)
                pref = self._pref_velocity(limit)
                pref = self._maybe_wait_aside(t, limit, pref)
            else:
                pref = (0.0, 0.0)
        self.last_limit = limit
        if self.waiting_zone is not None:
            self.stats.zone_wait_time += dt
        if cfg.motion == "swarmx" and self.status == TO_PICKUP and t >= self._next_resolve:
            here_zone = self.wh.zone_of(self.wh.cell_of(*self.pos))
            if here_zone and self.zones.stacked_conflict(here_zone, t):
                self._back_out(t, here_zone)

        walls = self.wh.nearby_wall_points(self.pos[0], self.pos[1], cfg.wall_dist,
                                           self.planner.blocked | self._foreign_zone_cells())
        r = cfg.radius + cfg.safety_margin
        if cfg.motion == "swarmx":
            if t < self._jitter_until:
                pref = (pref[0] + self._jitter[0], pref[1] + self._jitter[1])
            nbrs = self._neighbors(t)
            pref = self._keep_right(pref, nbrs)
            v = orca.compute_velocity(self.pos, self.vel, pref, r, cfg.max_speed, nbrs, walls,
                                      cfg.orca_horizon, cfg.orca_horizon_obst, max(dt, 0.05))
            v = self._emergency_filter(v, nbrs, dt)
            self._check_stuck(t, pref)
        else:
            self._baseline_deadlock_checks(t, dt)
            v = orca.compute_velocity(self.pos, self.vel, pref, r, cfg.max_speed, [], walls,
                                      cfg.orca_horizon, cfg.orca_horizon_obst, max(dt, 0.05))
        return v

    def _keep_right(self, pref, nbrs: Sequence[orca.Neighbor]):
        """Traffic rule: bias toward the right for oncoming robots (breaks ORCA symmetry)."""
        sp = math.hypot(*pref)
        if sp < 0.1:
            return pref
        hx, hy = pref[0] / sp, pref[1] / sp
        bias = 0.0
        for nb in nbrs:
            dx, dy = nb.pos[0] - self.pos[0], nb.pos[1] - self.pos[1]
            d = math.hypot(dx, dy)
            if d > 3.0 or d < 1e-6:
                continue
            fwd, lat = dx * hx + dy * hy, dx * -hy + dy * hx      # lat > 0: neighbour on my left
            nsp = math.hypot(*nb.vel)
            oncoming = nsp > 0.1 and (nb.vel[0] * hx + nb.vel[1] * hy) / nsp < -0.7
            if fwd > 0 and oncoming and abs(lat) < 0.9:
                bias = max(bias, 0.35 * (1.0 - d / 3.0) + 0.1)
        if bias == 0.0:
            return pref
        return (pref[0] + hy * bias, pref[1] - hx * bias)

    def _back_out(self, t: float, zid: str) -> None:
        """Leave a single-lane zone through the end I came in, then retry the pick."""
        tr = self._current_traversal()
        if tr is None or tr["entry"] is None:
            return
        out_cell = self.wh.zones[zid].ends[tr["entry"]]
        path = self.planner.astar(self._cell(), out_cell)
        if path is None:
            return
        self._next_resolve = t + 3.0
        self.route, self.route_idx = path, 0
        self.traversals = zone_traversals(self.wh, self.route)
        self.goal_idx, self.goal_kind, self.pickup_idx = len(self.route) - 1, "evade", None
        self._evade_deadline, self._evade_dwell = t + 8.0, 0.0
        self.zones.update(zid, phase="out")
        self.stats.deadlocks_resolved += 1
        self.events.append(f"{self.id} backs out of {zid} to let a leaving robot pass")

    def _maybe_wait_aside(self, t: float, limit: int, pref):
        """Don't queue on another aisle's mouth: wait on a free cell beside it instead."""
        if self.cfg.motion != "swarmx" or self.waiting_zone is None or not self.route:
            self._wait_spot = None
            return pref
        stop = self.route[min(limit, len(self.route) - 1)]
        if stop not in self._mouths or self.route_idx < limit - 1:
            return pref  # not yet at the queue point, or the queue point is harmless
        key = (self.waiting_zone, stop)
        if self._wait_spot is not None and self._wait_key != key:
            self._wait_spot = None  # a different queue than the one the spot was chosen for
        self._wait_key = key
        if self._wait_spot is None:
            occupied = {self.wh.cell_of(p.msg["x"], p.msg["y"]) for _, p in self.fresh_peers(t)}
            ahead = set(self.route[limit:limit + 4])
            best, bd = None, math.inf
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    c = (stop[0] + dx, stop[1] + dy)
                    if c == stop or c in occupied or c in ahead or c in self._mouths or not self.planner.passable(c) \
                            or self.wh.zone_of(c):
                        continue
                    d = math.dist(self.wh.center(c), self.pos) + (0.3 if dx and dy else 0.0)
                    if d < bd:
                        best, bd = c, d
            self._wait_spot = best or stop
        target = self.wh.center(self._wait_spot)
        dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        d = math.hypot(dx, dy)
        if d < 0.05:
            return (0.0, 0.0)
        sp = min(0.6, math.sqrt(2.0 * self.cfg.decel * d))
        return (dx / d * sp, dy / d * sp)

    def _foreign_zone_cells(self) -> Set[Cell]:
        """Cells of choke-point zones I do not hold: to ORCA they are walls.

        Enforces "no entry without a lock" at the motion level, so a waiting robot
        can be nudged aside by traffic but never shoved into a single-lane aisle.
        """
        if self.cfg.ghost:
            return set()
        here = self.wh.cell_of(*self.pos)
        here_zone = self.wh.zone_of(here)
        out = set()
        cx, cy = here
        for x in range(cx - 2, cx + 3):
            for y in range(cy - 2, cy + 3):
                z = self.wh.cell_zone.get((x, y))
                if z is not None and z != here_zone and not self.zones.holds(z):
                    out.add((x, y))
        return out

    def _emergency_filter(self, v, nbrs: Sequence[orca.Neighbor], dt: float):
        """Safe-following guard (RSS-style) applied after ORCA.

        ORCA assumes instantaneous velocity changes; real robots have bounded
        deceleration and peers' states arrive with latency. For every
        neighbour, cap my closing speed so that I can always stop inside the
        free gap even if the neighbour brakes as hard as possible.
        """
        a = self.cfg.decel * 1.2          # guaranteed braking deceleration (< platform max)
        tau = 0.2                          # reaction + communication latency
        vx, vy = v
        capped = False
        for nb in nbrs:
            dx, dy = nb.pos[0] - self.pos[0], nb.pos[1] - self.pos[1]
            d = math.hypot(dx, dy)
            if d < 1e-6 or d > 3.0:
                continue
            ux, uy = dx / d, dy / d
            mine = vx * ux + vy * uy
            if mine <= 0.0:
                continue                   # moving away / sideways
            theirs = nb.vel[0] * ux + nb.vel[1] * uy
            away = max(theirs, 0.0)       # they keep moving away while braking
            toward = max(-theirs, 0.0)    # they eat into the gap until they react and stop
            gap = (d - 2 * self.cfg.radius - 0.08 + away * away / (2 * a)
                   - toward * tau - toward * toward / (2 * a))
            vmax = 0.0 if gap <= 0 else a * (-tau + math.sqrt(tau * tau + 2 * gap / a))
            if mine > vmax:
                vx -= (mine - vmax) * ux
                vy -= (mine - vmax) * uy
                capped = True
        if capped:
            self.stats.estops += 1
        return (vx, vy)

    def _check_stuck(self, t: float, pref) -> None:
        """Livelock/deadlock safety net for dense crowds (rarely triggers)."""
        moving_intent = math.hypot(*pref) > 0.05 and self.waiting_zone is None and self.status not in STATIONARY
        remain = self._dist_along(self.goal_idx)
        t0, best = self._progress
        if not moving_intent or remain < best - 0.3 or remain > best + 3.0:
            self._progress = (t, remain)  # progressing, intentionally idle, or a new goal
            return
        if t - t0 > self.cfg.stuck_time:
            self._progress = (t, remain)
            if self.wh.zone_of(self.wh.cell_of(*self.pos)) is not None:
                return  # inside a single lane: never random-walk; the lock protocol will clear the way
            near = min(((math.hypot(p.msg["x"] - self.pos[0], p.msg["y"] - self.pos[1]), rid)
                        for rid, p in self.fresh_peers(t)), default=(math.inf, None))
            if near[0] < 2.0 and self._evade(t, near[1]):
                return
            self.stats.deadlocks_resolved += 1
            ang = self.rng.uniform(-math.pi, math.pi)
            self._jitter = (0.5 * math.cos(ang), 0.5 * math.sin(ang))
            self._jitter_until = t + 1.5
            avoid = set()
            for _, p in self.fresh_peers(t):
                if math.hypot(p.msg["x"] - self.pos[0], p.msg["y"] - self.pos[1]) < 2.0:
                    avoid.add(self.wh.cell_of(p.msg["x"], p.msg["y"]))
            if self._current_traversal() is None:
                self._replan(t, avoid=avoid)
            self.events.append(f"{self.id} resolved deadlock (replan+jitter)")
            self._progress = (t, remain)

    # ------------------------------------------ baseline: cell reservation
    def _body_cells(self) -> Set[Cell]:
        r = self.cfg.radius + 0.02
        x, y = self.pos
        out = set()
        for cx in range(math.floor(x - r), math.floor(x + r) + 1):
            for cy in range(math.floor(y - r), math.floor(y + r) + 1):
                qx, qy = min(max(x, cx), cx + 1.0), min(max(y, cy), cy + 1.0)
                if (qx - x) ** 2 + (qy - y) ** 2 < r * r:
                    out.add((cx, cy))
        return out

    def _reserve_cells(self, t: float, limit: int) -> int:
        """Stop-and-wait block control: drive only into cells we hold.

        Returns the furthest route index we hold (``route_idx - 1`` = must wait).
        """
        locks = self.cells
        key = lambda c: f"{c[0]},{c[1]}"  # noqa: E731
        keep = set()
        blocker = None
        intruding = False
        for c in self._body_cells():
            k = key(c)
            keep.add(k)
            if locks.holds(k):
                continue
            locks.claim(k, "X", t)
            if locks.can_enter(k, t):
                locks.hold(k)
            elif any(not b[0].startswith("<") for b in locks.blockers(k, t)):
                intruding = True  # overlapping a cell someone else holds: freeze until resolved
                blocker = next(b[0] for b in locks.blockers(k, t) if not b[0].startswith("<"))
        granted = self.route_idx - 1
        if self.route and limit >= 0 and not intruding:
            last = min(limit, self.route_idx + 2, len(self.route) - 1)
            for i in range(self.route_idx, last + 1):
                c = self.route[i]
                need = [c]
                if i > 0:
                    p = self.route[i - 1]
                    if p[0] != c[0] and p[1] != c[1]:  # diagonal move sweeps both side cells
                        need += [(c[0], p[1]), (p[0], c[1])]
                contested = []
                for n in need:
                    k = key(n)
                    if not locks.holds(k):
                        locks.claim(k, "X", t)
                        if not locks.can_enter(k, t):
                            contested.append(k)
                if contested:
                    really = {}
                    for k in contested:
                        real = [b for b in locks.blockers(k, t) if not b[0].startswith("<")]
                        if real:
                            really[k] = real[0][0]
                    if really:
                        blocker = next(iter(really.values()))
                        keep |= set(really)  # queue on truly contested cells, release the free ones
                    else:
                        keep |= {key(n) for n in need}  # just settling: keep the whole step
                    break
                for n in need:
                    locks.hold(key(n))
                keep |= {key(n) for n in need}
                granted = i
        for k in list(locks.mine):
            if k not in keep:
                locks.release(k)
        if blocker is None and self.waiting_zone is not None:
            zb = [b for b in self.zones.blockers(self.waiting_zone, t) if not b[0].startswith("<")]
            blocker = zb[0][0] if zb else None
        if blocker != self.blocked_by:
            self.blocked_by, self.blocked_since = blocker, t
        self._intruding = intruding
        return granted

    def _wait_cycle(self, t: float) -> Optional[List[str]]:
        chain, cur = [self.id], self.blocked_by
        while cur is not None and len(chain) < 10:
            if cur == self.id:
                return chain
            if cur in chain:
                return None
            p = self.peers.get(cur)
            if p is None or t - p.t_rx > self.cfg.peer_timeout:
                return None
            chain.append(cur)
            cur = p.msg.get("blocked_by")
        return None

    def _baseline_deadlock_checks(self, t: float, dt: float) -> None:
        if self.blocked_by is not None:
            self.stats.stop_time += dt
        # idle robots get out of the way of robots waiting for their cell
        if self.status in (IDLE, PARKING) and self.task is None and self.goal_kind != "evade":
            for rid, p in self.fresh_peers(t):
                if p.msg.get("blocked_by") == self.id and self.status == IDLE:
                    self._evade(t, rid)
                    return
        if self.blocked_by is None or self.goal_kind == "evade" or math.hypot(*self.vel) > 0.15:
            return  # grid AGVs only re-plan once stopped
        waited = t - self.blocked_since
        if waited < 1.0 or t < self._next_resolve:
            return
        cycle = self._wait_cycle(t)
        if cycle:
            def prio(rid):
                w = self.waiting_zone if rid == self.id else self.peers[rid].msg.get("wait")
                return (1 if w else 0, rid)
            if max(cycle, key=prio) == self.id:
                self._resolve_by_detour(t, cycle[1:])
            return
        other = self.peers.get(self.blocked_by)
        if other and (other.msg.get("st") in STATIONARY | {IDLE} and waited > 3.0 or waited > 10.0):
            self._resolve_by_detour(t, [self.blocked_by])

    def _resolve_by_detour(self, t: float, others: List[str]) -> None:
        self._next_resolve = t + 2.0
        if self._evade(t, others[0]):
            self.blocked_since = t
            return
        avoid: Set[Cell] = set()
        for rid in others:
            p = self.peers.get(rid)
            if p:
                avoid |= {tuple(int(v) for v in e[0].split(",")) for e in p.msg.get("cells", []) if e[3] == "hold"}
        avoid -= self._body_cells()
        old_next = self.route[self.route_idx] if self.route else None
        if self._current_traversal() is None and self._replan(t, avoid=avoid) and self.route and \
                len(self.route) > 1 and self.route[1] not in avoid and self.route[1] != old_next:
            self.stats.deadlocks_resolved += 1
            self.events.append(f"{self.id} detours around {others}")
        self.blocked_since = t
