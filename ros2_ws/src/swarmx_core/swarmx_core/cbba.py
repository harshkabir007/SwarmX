"""Consensus-Based Bundle Algorithm (CBBA) for decentralized task allocation.

Choi, Brunet, How - "Consensus-Based Decentralized Auctions for Robust Task
Allocation", IEEE T-RO 2009.

Each robot runs one :class:`CBBAAgent`. Phase 1 greedily builds a bundle of
up to ``max_bundle`` tasks (best insertion into the robot's own task path,
time-discounted reward). Phase 2 exchanges (bids ``y``, winners ``z``,
timestamps ``s``) with whoever is in radio range and applies the CBBA
decision table; lost tasks and everything bundled after them are released
and re-bid. There is no auctioneer.

Extensions for warehouse operation:
* **locking** - the task a robot is physically executing gets an unbeatable
  bid so it is never re-assigned mid-transport;
* **agent dropout** - tasks held by a silent robot are released after a
  timeout, so a failed robot's work is automatically re-allocated;
* **world changes** - :meth:`release_all` drops the robot's un-started bids
  (e.g. after a blocked aisle changes travel costs) and the fleet re-converges.
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Tuple

Cell = Tuple[int, int]
LOCKED_BID = 1e9
_EPS = 1e-6


class CBBAAgent:
    def __init__(self, agent_id: str, travel_time: Callable[[Cell, Cell], float],
                 service_time: float = 2.0, max_bundle: int = 3, discount: float = 0.99,
                 reward: float = 100.0, agent_timeout: float = 4.0):
        self.id = agent_id
        self.travel_time = travel_time
        self.service_time = service_time
        self.max_bundle = max_bundle
        self.discount = discount
        self.reward = reward
        self.agent_timeout = agent_timeout

        self.tasks: Dict[str, dict] = {}
        self.done: Dict[str, float] = {}
        self.y: Dict[str, float] = {}
        self.z: Dict[str, Optional[str]] = {}
        self.s: Dict[str, float] = {agent_id: 0.0}
        self.bundle: List[str] = []
        self.path: List[str] = []
        self.locked: Optional[str] = None
        self.locked_picked = False
        self.enabled = True
        self.start: Cell = (0, 0)
        self.changed = True
        self.rebuilds = 0
        self.dead: Dict[str, float] = {}  # tombstones: agent -> last timestamp seen before expiry
        self.skip: set = set()            # tasks this agent cannot serve right now (unreachable)

    # ------------------------------------------------------------ task table
    def add_task(self, task: dict) -> bool:
        tid = task["id"]
        if tid in self.tasks or tid in self.done:
            return False
        self.tasks[tid] = task
        self.y.setdefault(tid, 0.0)
        self.z.setdefault(tid, None)
        self.changed = True
        return True

    def abandon(self, tid: str) -> None:
        """Give a task back to the auction (e.g. its pick face became unreachable)."""
        if self.locked == tid:
            self.locked = None
            self.locked_picked = False
        if self.z.get(tid) == self.id:
            self.y[tid], self.z[tid] = 0.0, None
        if tid in self.bundle:
            idx = self.bundle.index(tid)
            for later in self.bundle[idx + 1:]:
                if self.z.get(later) == self.id:
                    self.y[later], self.z[later] = 0.0, None
            self.bundle = self.bundle[:idx]
        self.path = [p for p in self.path if p in self.bundle]
        self.skip.add(tid)
        self.changed = True

    def mark_done(self, tid: str, t: float) -> None:
        self.done[tid] = t
        self.tasks.pop(tid, None)
        self.y.pop(tid, None)
        self.z.pop(tid, None)
        if tid in self.bundle:
            self.bundle.remove(tid)
        if tid in self.path:
            self.path.remove(tid)
        if self.locked == tid:
            self.locked = None
            self.locked_picked = False
        self.changed = True

    def lock(self, tid: str, picked: bool = False) -> None:
        if tid not in self.tasks:
            return
        self.locked = tid
        self.locked_picked = picked
        if tid not in self.bundle:
            self.bundle.insert(0, tid)
        if tid in self.path:
            self.path.remove(tid)
        self.path.insert(0, tid)
        self.y[tid] = LOCKED_BID
        self.z[tid] = self.id
        self.changed = True

    def release_all(self, keep_locked: bool = True) -> None:
        """Withdraw every un-started bid so the fleet can re-allocate."""
        keep = self.locked if keep_locked else None
        self.skip.clear()  # the world changed: re-evaluate everything
        for tid in list(self.bundle):
            if tid != keep and self.z.get(tid) == self.id:
                self.y[tid] = 0.0
                self.z[tid] = None
        self.bundle = [keep] if keep else []
        self.path = [keep] if keep else []
        if not keep_locked:
            self.locked = None
        self.changed = True

    # --------------------------------------------------------------- scoring
    def _path_score(self, path: List[str]) -> float:
        t = 0.0
        cur = self.start
        score = 0.0
        for tid in path:
            task = self.tasks[tid]
            pick, drop = tuple(task["pickup"]), tuple(task["dropoff"])
            if tid == self.locked and self.locked_picked:
                t += self.travel_time(cur, drop) + self.service_time
            else:
                t += self.travel_time(cur, pick) + self.service_time
                t += self.travel_time(pick, drop) + self.service_time
            if math.isinf(t):
                return -math.inf
            cur = drop
            score += self.reward * task.get("priority", 1.0) * (self.discount ** t)
        return score

    def _beats(self, bid: float, bidder: Optional[str], other_bid: float, other: Optional[str]) -> bool:
        if bid > other_bid + _EPS:
            return True
        if abs(bid - other_bid) <= _EPS and bidder is not None and (other is None or bidder < other):
            return True
        return False

    # ---------------------------------------------------- phase 1: bundling
    def build_bundle(self) -> bool:
        if not self.enabled:
            return False
        added = False
        base = self._path_score(self.path)
        while len(self.bundle) < self.max_bundle:
            best: Optional[Tuple[float, str, int]] = None
            for tid in self.tasks:
                if tid in self.bundle or tid in self.skip:
                    continue
                first = 1 if (self.locked and self.path and self.path[0] == self.locked) else 0
                best_c, best_n = -math.inf, -1
                for n in range(first, len(self.path) + 1):
                    cand = self.path[:n] + [tid] + self.path[n:]
                    c = self._path_score(cand) - base
                    if c > best_c:
                        best_c, best_n = c, n
                if best_c <= _EPS or math.isinf(best_c):
                    continue
                if not self._beats(best_c, self.id, self.y.get(tid, 0.0), self.z.get(tid)):
                    continue
                if best is None or best_c > best[0] + _EPS or (abs(best_c - best[0]) <= _EPS and tid < best[1]):
                    best = (best_c, tid, best_n)
            if best is None:
                break
            c, tid, n = best
            self.bundle.append(tid)
            self.path.insert(n, tid)
            self.y[tid] = c
            self.z[tid] = self.id
            base = self._path_score(self.path)
            added = True
        if added:
            self.changed = True
            self.rebuilds += 1
        return added

    # --------------------------------------------------- phase 2: consensus
    def message_fields(self, now: float) -> dict:
        self.s[self.id] = now
        return {
            "y": {k: round(v, 6) for k, v in self.y.items()},
            "z": dict(self.z),
            "s": dict(self.s),
            "tasks": self.tasks,
            "done": list(self.done.keys())[-64:],
        }

    def consensus(self, msg: dict, now: float) -> bool:
        k = msg["src"]
        if k == self.id:
            return False
        for tid in msg.get("done", ()):
            if tid not in self.done:
                self.mark_done(tid, now)
        for tid, spec in msg.get("tasks", {}).items():
            self.add_task(spec)

        yk: Dict[str, float] = msg.get("y", {})
        zk: Dict[str, Optional[str]] = msg.get("z", {})
        sk: Dict[str, float] = msg.get("s", {})
        me = self.id
        si = self.s
        changed = False
        # an agent we declared dead is alive again if it talks to us or someone has fresher news
        self.dead.pop(k, None)
        for m, t in sk.items():
            if m in self.dead and t > self.dead[m]:
                del self.dead[m]

        def s_newer(m: Optional[str]) -> bool:
            return m is not None and sk.get(m, -1.0) > si.get(m, -1.0)

        for tid in list(self.tasks.keys()):
            if tid not in zk:
                continue
            z_k, y_k = zk[tid], yk.get(tid, 0.0)
            z_i, y_i = self.z.get(tid), self.y.get(tid, 0.0)
            action = "leave"
            if z_k == k:
                if z_i == me:
                    action = "update" if self._beats(y_k, k, y_i, me) else "leave"
                elif z_i == k:
                    action = "update"
                elif z_i is None:
                    action = "update"
                else:  # z_i = m
                    action = "update" if (s_newer(z_i) or self._beats(y_k, k, y_i, z_i)) else "leave"
            elif z_k == me:
                if z_i == me:
                    action = "leave"
                elif z_i == k:
                    action = "reset"
                elif z_i is None:
                    action = "leave"
                else:
                    action = "reset" if s_newer(z_i) else "leave"
            elif z_k is not None:  # z_k = m, not k and not me
                m = z_k
                if z_i == me:
                    action = "update" if (s_newer(m) and self._beats(y_k, m, y_i, me)) else "leave"
                elif z_i == k:
                    action = "update" if s_newer(m) else "reset"
                elif z_i == m:
                    action = "update" if s_newer(m) else "leave"
                elif z_i is None:
                    action = "update" if s_newer(m) else "leave"
                else:  # z_i = n, n not in {me, k, m}
                    n = z_i
                    if s_newer(m) and s_newer(n):
                        action = "update"
                    elif s_newer(m) and self._beats(y_k, m, y_i, n):
                        action = "update"
                    elif s_newer(n) and si.get(m, -1.0) > sk.get(m, -1.0):
                        action = "reset"
            else:  # z_k is None
                if z_i == me:
                    action = "leave"
                elif z_i == k:
                    action = "update"
                elif z_i is None:
                    action = "leave"
                else:
                    action = "update" if s_newer(z_i) else "leave"

            if tid == self.locked:
                action = "leave"  # we are physically executing it
            elif action == "update" and z_k in self.dead:
                action = "reset"  # stale gossip about a failed robot
            if action == "update":
                if self.z.get(tid) != z_k or abs(self.y.get(tid, 0.0) - y_k) > _EPS:
                    changed = True
                self.y[tid], self.z[tid] = y_k, z_k
            elif action == "reset":
                if self.z.get(tid) is not None:
                    changed = True
                self.y[tid], self.z[tid] = 0.0, None

        # timestamps
        si[k] = now
        for m, t in sk.items():
            if m != me and m not in self.dead and t > si.get(m, -1.0):
                si[m] = t

        if self._drop_lost_tasks():
            changed = True
        if changed:
            self.changed = True
        return changed

    def _drop_lost_tasks(self) -> bool:
        for idx, tid in enumerate(self.bundle):
            if self.z.get(tid) != self.id:
                for later in self.bundle[idx + 1:]:
                    if self.z.get(later) == self.id and later != self.locked:
                        self.y[later] = 0.0
                        self.z[later] = None
                self.bundle = self.bundle[:idx]
                self.path = [p for p in self.path if p in self.bundle]
                return True
        return False

    def expire_agents(self, now: float) -> List[str]:
        """Release tasks won by robots we have not heard from (failure recovery)."""
        dead = [a for a, t in self.s.items() if a != self.id and now - t > self.agent_timeout]
        if not dead:
            return []
        for tid, w in self.z.items():
            if w in dead:
                self.y[tid] = 0.0
                self.z[tid] = None
                self.changed = True
        for a in dead:
            self.dead[a] = self.s.pop(a)
        return dead

    # ------------------------------------------------------------- queries
    def current_task(self) -> Optional[str]:
        return self.path[0] if self.path else None

    def assignment(self) -> Dict[str, Optional[str]]:
        return dict(self.z)
