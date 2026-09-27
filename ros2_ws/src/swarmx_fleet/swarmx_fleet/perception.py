"""Lidar perception for SwarmX (ROS-agnostic, unit-testable).

Turns 2D laser scans into the three inputs ``FleetAgent.observe`` expects:

* **blocked cells** - floor cells where lidar returns persist although the
  static map says "free" (a pallet in an aisle, a dead robot) and that are not
  explained by a robot we are in radio contact with;
* **free cells** - floor cells the beams passed through (used to clear old
  obstacle reports);
* **robot-like clusters** - compact return clusters not matched to any live
  peer: non-cooperative bodies that ORCA must avoid with full responsibility.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Sequence, Set, Tuple

from swarmx_core.warehouse import Cell, Warehouse


class ScanPerception:
    def __init__(self, warehouse: Warehouse, max_range: float = 5.0, confirm: int = 4, clear: int = 5,
                 peer_clearance: float = 0.6, map_margin: float = 0.2, step: float = 0.25):
        self.wh = warehouse
        self.max_range = max_range
        self.confirm = confirm
        self.clear = clear
        self.peer_clearance = peer_clearance
        self.map_margin = map_margin
        self.step = step
        self._hits: Dict[Cell, int] = {}
        self._misses: Dict[Cell, int] = {}
        self.blocked: Set[Cell] = set()

    def _near_structure(self, x: float, y: float) -> bool:
        """Return explained by a shelf/wall (tolerates small localization error)."""
        m = self.map_margin
        for dx in (-m, 0.0, m):
            for dy in (-m, 0.0, m):
                if not self.wh.is_free(self.wh.cell_of(x + dx, y + dy)):
                    return True
        return False

    def process(self, pose: Tuple[float, float, float], ranges: Sequence[float], angle_min: float,
                angle_increment: float, range_min: float, range_max: float,
                peers: Iterable[Tuple[float, float]] = ()) -> Tuple[Set[Cell], Set[Cell], List[Tuple[float, float]]]:
        x0, y0, th = pose
        peers = list(peers)
        rmax = min(range_max, self.max_range)
        hit_cells: Set[Cell] = set()
        seen_free: Set[Cell] = set()
        points: List[Tuple[float, float]] = []
        for i, r in enumerate(ranges):
            a = th + angle_min + i * angle_increment
            ca, sa = math.cos(a), math.sin(a)
            hit = math.isfinite(r) and range_min <= r <= rmax
            reach = (r - 0.3) if hit else rmax
            d = self.step
            while d < reach:
                c = self.wh.cell_of(x0 + ca * d, y0 + sa * d)
                if not self.wh.is_free(c):
                    break
                seen_free.add(c)
                d += self.step
            if not hit:
                continue
            hx, hy = x0 + ca * r, y0 + sa * r
            if self._near_structure(hx, hy):
                continue
            if any(math.hypot(hx - px, hy - py) < self.peer_clearance for px, py in peers):
                continue  # a robot we are talking to - ORCA handles it via P2P state
            points.append((hx, hy))
            hit_cells.add(self.wh.cell_of(hx, hy))
        # hysteresis so a single spurious return never re-routes the fleet
        for c in hit_cells:
            self._hits[c] = self._hits.get(c, 0) + 1
            self._misses.pop(c, None)
        for c in list(self._hits):
            if c not in hit_cells:
                self._hits[c] -= 1
                if self._hits[c] <= 0:
                    del self._hits[c]
        for c, n in self._hits.items():
            if n >= self.confirm:
                self.blocked.add(c)
        free: Set[Cell] = set()
        for c in seen_free - hit_cells:
            if c in self.blocked:
                self._misses[c] = self._misses.get(c, 0) + 1
                if self._misses[c] >= self.clear:
                    self.blocked.discard(c)
                    self._misses.pop(c, None)
                    free.add(c)
            else:
                free.add(c)
        return set(self.blocked), free, _clusters(points, sensor=(x0, y0))


def _clusters(points: List[Tuple[float, float]], gap: float = 0.25, max_extent: float = 0.9,
              sensor: Tuple[float, float] = (0.0, 0.0), body_radius: float = 0.25) -> List[Tuple[float, float]]:
    """Group consecutive returns into compact clusters; return robot-sized body centres.

    Returns lie on the near surface of a body, so each centroid is pushed away
    from the sensor by roughly one body radius.
    """
    out, cur = [], []
    for p in points:
        if cur and math.dist(cur[-1], p) > gap:
            out.append(cur)
            cur = []
        cur.append(p)
    if cur:
        out.append(cur)
    centres = []
    for cl in out:
        if len(cl) < 3:
            continue
        if math.dist(cl[0], cl[-1]) > max_extent:
            continue
        cx = sum(p[0] for p in cl) / len(cl)
        cy = sum(p[1] for p in cl) / len(cl)
        d = math.hypot(cx - sensor[0], cy - sensor[1])
        if d > 1e-6:
            cx += (cx - sensor[0]) / d * body_radius
            cy += (cy - sensor[1]) / d * body_radius
        centres.append((cx, cy))
    return centres
