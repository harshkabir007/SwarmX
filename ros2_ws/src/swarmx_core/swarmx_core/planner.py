"""Grid path planning for edge hardware.

* 8-connected A* with corner-cutting protection, per-cell extra cost and
  per-zone entry penalties (used for congestion-aware re-routing).
* Cached Dijkstra distance fields for O(1) travel-time estimates, which the
  CBBA task allocator queries thousands of times per bundle build.

Everything is pure Python and allocation-light so it runs on a Raspberry Pi.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .warehouse import Cell, Warehouse

SQRT2 = math.sqrt(2.0)
_NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


class GridPlanner:
    def __init__(self, warehouse: Warehouse):
        self.wh = warehouse
        self.blocked: Set[Cell] = set()
        self.version = 0
        self._fields: Dict[Cell, Dict[Cell, float]] = {}
        self.lanes: Dict[Cell, int] = {}  # optional one-way lanes: cell -> allowed x direction
        self.allow_diagonal = True

    # ----------------------------------------------------------- world model
    def set_blocked(self, cells: Iterable[Cell], blocked: bool = True) -> bool:
        changed = False
        for c in cells:
            c = (int(c[0]), int(c[1]))
            if blocked and c not in self.blocked and self.wh.is_free(c):
                self.blocked.add(c)
                changed = True
            elif not blocked and c in self.blocked:
                self.blocked.discard(c)
                changed = True
        if changed:
            self.version += 1
            self._fields.clear()
        return changed

    def passable(self, c: Cell) -> bool:
        return self.wh.is_free(c) and c not in self.blocked

    def _edge_ok(self, c: Cell, n: Cell) -> bool:
        if not self.passable(n):
            return False
        dx, dy = n[0] - c[0], n[1] - c[1]
        if dx and dy and not self.allow_diagonal:
            return False
        if dx and dy and not (self.passable((c[0] + dx, c[1])) and self.passable((c[0], c[1] + dy))):
            return False  # no corner cutting past shelves
        if dx and self.lanes and (self.lanes.get(c, 0) * dx < 0 or self.lanes.get(n, 0) * dx < 0):
            return False  # against a one-way lane
        return True

    def neighbors(self, c: Cell):
        x, y = c
        for dx, dy, cost in _NEIGHBORS:
            n = (x + dx, y + dy)
            if self._edge_ok(c, n):
                yield n, cost

    def predecessors(self, c: Cell):
        x, y = c
        for dx, dy, cost in _NEIGHBORS:
            p = (x - dx, y - dy)
            if self.passable(p) and self._edge_ok(p, c):
                yield p, cost

    # ------------------------------------------------------------------ A*
    def astar(self, start: Cell, goal: Cell,
              zone_penalty=None,
              cell_cost: Optional[Dict[Cell, float]] = None,
              avoid: Optional[Set[Cell]] = None) -> Optional[List[Cell]]:
        if start == goal:
            return [start]
        if not self.passable(goal):
            return None
        zone_penalty = zone_penalty or {}
        pen_fn = zone_penalty if callable(zone_penalty) else None
        zones = self.wh.zones
        cell_cost = cell_cost or {}
        avoid = avoid or set()
        zone_of = self.wh.cell_zone.get
        gx, gy = goal

        def h(c: Cell) -> float:  # octile distance
            dx, dy = abs(c[0] - gx), abs(c[1] - gy)
            return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)

        open_heap = [(h(start), 0.0, start)]
        g = {start: 0.0}
        parent: Dict[Cell, Cell] = {}
        closed: Set[Cell] = set()
        while open_heap:
            _, gc, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            if cur == goal:
                path = [cur]
                while cur in parent:
                    cur = parent[cur]
                    path.append(cur)
                return path[::-1]
            closed.add(cur)
            zc = zone_of(cur)
            for n, step in self.neighbors(cur):
                if n in avoid and n != goal:
                    continue
                if avoid and n[0] != cur[0] and n[1] != cur[1] and \
                        ((n[0], cur[1]) in avoid or (cur[0], n[1]) in avoid):
                    continue  # diagonal would sweep an avoided cell
                cost = gc + step + cell_cost.get(n, 0.0)
                zn = zone_of(n)
                if zn is not None and zn != zc:
                    # entering a choke point: static load penalty, or a space-time
                    # penalty callback(zone, entry_end, metres_so_far, goal_inside)
                    cost += pen_fn(zn, zones[zn].end_of(cur), gc, zone_of(goal) == zn) if pen_fn \
                        else zone_penalty.get(zn, 0.0)
                if cost < g.get(n, math.inf):
                    g[n] = cost
                    parent[n] = cur
                    heapq.heappush(open_heap, (cost + h(n), cost, n))
        return None

    def plan_via(self, start: Cell, waypoints: Sequence[Cell], **kw) -> Optional[List[Cell]]:
        """Concatenated A* through ``waypoints`` (e.g. pickup then drop-off)."""
        path = [start]
        cur = start
        for wp in waypoints:
            leg = self.astar(cur, wp, **kw)
            if leg is None:
                return None
            path.extend(leg[1:])
            cur = wp
        return path

    # ------------------------------------------------------- distance field
    def distance_field(self, target: Cell) -> Dict[Cell, float]:
        f = self._fields.get(target)
        if f is not None:
            return f
        f = {target: 0.0}
        heap = [(0.0, target)]
        while heap:
            d, c = heapq.heappop(heap)
            if d > f.get(c, math.inf):
                continue
            for n, step in self.predecessors(c):  # distances *to* target on a directed graph
                nd = d + step
                if nd < f.get(n, math.inf):
                    f[n] = nd
                    heapq.heappush(heap, (nd, n))
        self._fields[target] = f
        return f

    def distance(self, a: Cell, b: Cell) -> float:
        """Shortest path length in metres (inf if unreachable)."""
        return self.distance_field(b).get(a, math.inf)


def path_length(path: Sequence[Cell]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def zone_traversals(wh: Warehouse, path: Sequence[Cell]) -> List[dict]:
    """Split a path into the choke-point zones it passes through.

    Returns dicts ``{zone, start, end, entry, exit, mode, lo, hi}``: ``start``/``end``
    are path indices of the first/last cell inside the zone, ``entry``/``exit``
    the end labels ("S"/"N"), ``mode`` = entry+exit ("SN", "NS", "SS", "NN", or
    "XX" when an end is unknown) and ``lo``/``hi`` the zone-local cell interval
    the robot occupies (index 0 = S end).
    """
    out: List[dict] = []
    i, n = 0, len(path)
    while i < n:
        zid = wh.cell_zone.get(path[i])
        if zid is None:
            i += 1
            continue
        j = i
        while j + 1 < n and wh.cell_zone.get(path[j + 1]) == zid:
            j += 1
        zone = wh.zones[zid]
        entry = zone.end_of(path[i - 1]) if i > 0 else None
        exit_ = zone.end_of(path[j + 1]) if j + 1 < n else None
        mode = entry + exit_ if entry and exit_ else "XX"
        local = [zone.cells.index(c) for c in path[i:j + 1]]
        out.append({"zone": zid, "start": i, "end": j, "entry": entry, "exit": exit_, "mode": mode,
                    "lo": min(local), "hi": max(local)})
        i = j + 1
    return out
