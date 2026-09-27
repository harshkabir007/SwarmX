"""FleetMonitor - builds the dashboard's view of the fleet from P2P traffic only.

The monitor is a *passive peer*: it listens to the same broadcasts robots
exchange with each other and never commands motion. If it (or the laptop it
runs on) disappears, the fleet keeps working. The same class is fed by the
in-process sim bus, UDP multicast, Zenoh, or the ROS 2 bridge node.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque, Dict, List, Optional

from .. import protocol


class FleetMonitor:
    def __init__(self, stale_after: float = 3.0, history_s: float = 900.0, block_ttl: float = 60.0):
        self.stale_after = stale_after
        self.block_ttl = block_ttl
        self.robots: Dict[str, dict] = {}
        self.rx_time: Dict[str, float] = {}
        self.tasks: Dict[str, dict] = {}
        self.done: Dict[str, dict] = {}         # tid -> {"t": time, "by": robot}
        self.blocked: Dict[tuple, float] = {}   # cell -> last report time
        self.events: Deque[dict] = deque(maxlen=80)
        self.timeline: Deque[List[float]] = deque(maxlen=int(history_s))
        self._last_sample = -1e9
        self._msg_times: Deque[tuple] = deque()  # (t, bytes)
        self.t = 0.0
        self.t_start: Optional[float] = None
        self.lock = threading.Lock()

    # ------------------------------------------------------------- ingest
    def ingest(self, msg: dict, now: Optional[float] = None, size: Optional[int] = None) -> None:
        now = time.time() if now is None else now
        with self.lock:
            self.t = max(self.t, now)
            if self.t_start is None:
                self.t_start = now
                self.timeline.append([round(now, 1), 0])
                self._last_sample = now
            if now - self._last_sample >= 1.0:
                self._last_sample = now
                self.timeline.append([round(now, 1), len(self.done)])
            self._msg_times.append((now, size if size is not None else len(protocol.encode(msg))))
            typ, src = msg.get("type"), msg.get("src")
            if typ == protocol.STATE:
                prev = self.robots.get(src)
                st = msg.get("st")
                if prev is None:
                    self._event(now, src, f"{src} joined the network", kind="info")
                elif prev.get("st") != st and st in ("failed", "to_charger"):
                    self._event(now, src, f"{src} failed" if st == "failed" else f"{src} battery low, heading to a charger",
                                kind="critical" if st == "failed" else "alert")
                elif prev.get("st") == "failed" and st != "failed":
                    self._event(now, src, f"{src} back in service", kind="info")
                for text in msg.get("ev", ()):
                    self._event(now, src, text, kind="alert" if ("deadlock" in text or "evades" in text) else "info")
                self.robots[src] = msg
                self.rx_time[src] = now
            elif typ == protocol.TASK:
                for spec in msg.get("tasks", [msg.get("task")]):
                    if spec and spec["id"] not in self.tasks and spec["id"] not in self.done:
                        self.tasks[spec["id"]] = spec
            elif typ == protocol.CBBA:
                for tid, spec in msg.get("tasks", {}).items():
                    if tid not in self.tasks and tid not in self.done:
                        self.tasks[tid] = spec
                for tid in msg.get("done", ()):
                    self._mark_done(tid, now, None)
            elif typ == protocol.TASK_DONE:
                self._mark_done(msg["task"], now, src)
            elif typ == protocol.BLOCKED:
                cells = [tuple(c) for c in msg.get("cells", [])]
                cleared = [tuple(c) for c in msg.get("cleared", [])]
                if cells:
                    self._event(now, src, f"{src} reports obstacle at {cells}", kind="alert")
                if cleared:
                    self._event(now, src, f"{src} reports {cleared} clear again", kind="info")
                for c in cells:
                    self.blocked[c] = now
                for c in cleared:
                    self.blocked.pop(c, None)

    def _mark_done(self, tid: str, now: float, by: Optional[str]) -> None:
        if tid in self.done:
            if by and not self.done[tid].get("by"):
                self.done[tid]["by"] = by
            return
        self.done[tid] = {"t": now, "by": by}
        spec = self.tasks.pop(tid, None)
        if by:
            self._event(now, by, f"{by} delivered {tid}", kind="done")
        elif spec is not None:
            self._event(now, "", f"{tid} delivered", kind="done")

    def _event(self, t: float, src: str, text: str, kind: str = "info") -> None:
        self.events.append({"t": round(t, 2), "src": src, "text": text, "kind": kind})

    def note(self, text: str, kind: str = "info", now: Optional[float] = None) -> None:
        """External event (operator command, sim ground truth)."""
        with self.lock:
            self._event(time.time() if now is None else now, "", text, kind)

    # ----------------------------------------------------------- snapshot
    def snapshot(self, now: Optional[float] = None) -> dict:
        now = self.t if now is None else now
        with self.lock:
            while self._msg_times and now - self._msg_times[0][0] > 5.0:
                self._msg_times.popleft()
            span = 5.0 if self._msg_times else 1.0
            msgs_s = len(self._msg_times) / span
            kbps = sum(b for _, b in self._msg_times) / 1024.0 / span
            robots = []
            assigned: Dict[str, str] = {}
            for rid in sorted(self.robots, key=_robot_key):
                m = self.robots[rid]
                age = now - self.rx_time.get(rid, now)
                r = {k: m.get(k) for k in ("x", "y", "th", "vx", "vy", "bat", "st", "task", "goal", "path",
                                           "zones", "wait", "mode", "alloc", "stats", "bundle", "locked")}
                r["id"] = rid
                r["age"] = round(age, 2)
                r["alive"] = age < self.stale_after and m.get("st") != "failed"
                robots.append(r)
                for tid in m.get("bundle") or []:
                    assigned.setdefault(tid, rid)
                if m.get("task"):
                    assigned[m["task"]] = rid
            tasks = []
            for tid, spec in sorted(self.tasks.items()):
                tasks.append({"id": tid, "pickup": spec["pickup"], "dropoff": spec["dropoff"],
                              "robot": assigned.get(tid), "state": "assigned" if tid in assigned else "pending"})
            window = [d for d in self.done.values() if now - d["t"] <= 60.0]
            alive = [r for r in robots if r["alive"]]
            bats = [r["bat"] for r in alive if r.get("bat") is not None]
            return {
                "t": round(now, 2),
                "t_start": self.t_start if self.t_start is not None else now,
                "robots": robots,
                "tasks": tasks,
                "blocked": sorted(list(c) for c, ts in self.blocked.items() if now - ts <= self.block_ttl),
                "metrics": {
                    "delivered": len(self.done),
                    "pending": sum(1 for t in tasks if t["state"] == "pending"),
                    "assigned": sum(1 for t in tasks if t["state"] == "assigned"),
                    "throughput_per_min": len(window),
                    "robots_alive": len(alive),
                    "robots_total": len(robots),
                    "battery_avg": round(sum(bats) / len(bats), 3) if bats else None,
                    "msgs_per_s": round(msgs_s, 1),
                    "kbytes_per_s": round(kbps, 2),
                },
                "timeline": list(self.timeline),
                "events": list(self.events)[-40:],
            }


def _robot_key(rid: str):
    digits = "".join(ch for ch in rid if ch.isdigit())
    return (rid.rstrip("0123456789"), int(digits) if digits else 0)
