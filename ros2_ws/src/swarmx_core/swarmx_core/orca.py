"""Optimal Reciprocal Collision Avoidance (ORCA / RVO2) in pure Python.

A faithful port of the agent-agent part of the RVO2 library
(van den Berg, Guy, Lin, Manocha - "Reciprocal n-body Collision Avoidance",
ISRR 2011), including the three linear programs. Static obstacles are handled
as hard half-plane constraints built from the closest point of each nearby
obstacle cell, which is exact for the axis-aligned shelf faces of a warehouse.

No numpy: 2D tuples are faster than small arrays on a Raspberry Pi.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Vec = Tuple[float, float]
EPS = 1e-5


def _det(a: Vec, b: Vec) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1])


def _add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1])


def _mul(a: Vec, s: float) -> Vec:
    return (a[0] * s, a[1] * s)


def _abs_sq(a: Vec) -> float:
    return a[0] * a[0] + a[1] * a[1]


def _norm(a: Vec) -> Vec:
    n = math.sqrt(_abs_sq(a))
    return (a[0] / n, a[1] / n) if n > EPS else (0.0, 0.0)


@dataclass
class Line:
    """Half-plane: permitted velocities lie to the left of ``direction``."""
    point: Vec
    direction: Vec


@dataclass
class Neighbor:
    pos: Vec
    vel: Vec
    radius: float
    responsibility: float = 0.5  # 0.5 = reciprocal, 1.0 = neighbour will not avoid us


def _lp1(lines: Sequence[Line], line_no: int, radius: float, opt: Vec, direction_opt: bool
         ) -> Optional[Vec]:
    ln = lines[line_no]
    dot = _dot(ln.point, ln.direction)
    disc = dot * dot + radius * radius - _abs_sq(ln.point)
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    t_left, t_right = -dot - sq, -dot + sq
    for i in range(line_no):
        denom = _det(ln.direction, lines[i].direction)
        numer = _det(lines[i].direction, _sub(ln.point, lines[i].point))
        if abs(denom) <= EPS:
            if numer < 0.0:
                return None
            continue
        t = numer / denom
        if denom >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return None
    if direction_opt:
        t = t_right if _dot(opt, ln.direction) > 0.0 else t_left
    else:
        t = _dot(ln.direction, _sub(opt, ln.point))
        t = min(max(t, t_left), t_right)
    return _add(ln.point, _mul(ln.direction, t))


def _lp2(lines: Sequence[Line], radius: float, opt: Vec, direction_opt: bool) -> Tuple[int, Vec]:
    if direction_opt:
        result = _mul(opt, radius)
    elif _abs_sq(opt) > radius * radius:
        result = _mul(_norm(opt), radius)
    else:
        result = opt
    for i, ln in enumerate(lines):
        if _det(ln.direction, _sub(ln.point, result)) > 0.0:
            r = _lp1(lines, i, radius, opt, direction_opt)
            if r is None:
                return i, result
            result = r
    return len(lines), result


def _lp3(lines: Sequence[Line], n_obst: int, begin: int, radius: float, result: Vec) -> Vec:
    distance = 0.0
    for i in range(begin, len(lines)):
        li = lines[i]
        if _det(li.direction, _sub(li.point, result)) > distance:
            proj = list(lines[:n_obst])
            for j in range(n_obst, i):
                lj = lines[j]
                determinant = _det(li.direction, lj.direction)
                if abs(determinant) <= EPS:
                    if _dot(li.direction, lj.direction) > 0.0:
                        continue
                    point = _mul(_add(li.point, lj.point), 0.5)
                else:
                    point = _add(li.point, _mul(li.direction,
                                                _det(lj.direction, _sub(li.point, lj.point)) / determinant))
                proj.append(Line(point, _norm(_sub(lj.direction, li.direction))))
            fail, r = _lp2(proj, radius, (-li.direction[1], li.direction[0]), True)
            if fail >= len(proj):
                result = r
            distance = _det(li.direction, _sub(li.point, result))
    return result


def compute_velocity(pos: Vec, vel: Vec, pref_vel: Vec, radius: float, max_speed: float,
                     neighbors: Sequence[Neighbor], wall_points: Sequence[Vec] = (),
                     time_horizon: float = 2.0, time_horizon_obst: float = 0.6,
                     time_step: float = 0.1) -> Vec:
    """Return the collision-free velocity closest to ``pref_vel``."""
    lines: List[Line] = []

    # --- static obstacles: hard constraints  n.v <= (d - r) / tau_obst
    for wp in wall_points:
        rel = _sub(wp, pos)
        d = math.sqrt(_abs_sq(rel))
        if d < EPS:
            continue
        n = (rel[0] / d, rel[1] / d)
        c = (d - radius) / time_horizon_obst
        lines.append(Line(_mul(n, c), (-n[1], n[0])))
    n_obst = len(lines)

    # --- other robots: reciprocal velocity obstacles
    inv_th = 1.0 / time_horizon
    for nb in neighbors:
        rel_pos = _sub(nb.pos, pos)
        rel_vel = _sub(vel, nb.vel)
        dist_sq = _abs_sq(rel_pos)
        comb_r = radius + nb.radius
        comb_r_sq = comb_r * comb_r
        if dist_sq > comb_r_sq:
            w = _sub(rel_vel, _mul(rel_pos, inv_th))
            w_len_sq = _abs_sq(w)
            dot1 = _dot(w, rel_pos)
            if dot1 < 0.0 and dot1 * dot1 > comb_r_sq * w_len_sq:
                w_len = math.sqrt(w_len_sq)
                unit_w = _mul(w, 1.0 / w_len)
                direction = (unit_w[1], -unit_w[0])
                u = _mul(unit_w, comb_r * inv_th - w_len)
            else:
                leg = math.sqrt(dist_sq - comb_r_sq)
                if _det(rel_pos, w) > 0.0:
                    direction = ((rel_pos[0] * leg - rel_pos[1] * comb_r) / dist_sq,
                                 (rel_pos[0] * comb_r + rel_pos[1] * leg) / dist_sq)
                else:
                    direction = (-(rel_pos[0] * leg + rel_pos[1] * comb_r) / dist_sq,
                                 -(-rel_pos[0] * comb_r + rel_pos[1] * leg) / dist_sq)
                dot2 = _dot(rel_vel, direction)
                u = _sub(_mul(direction, dot2), rel_vel)
        else:
            # already overlapping: resolve within one time step
            inv_ts = 1.0 / time_step
            w = _sub(rel_vel, _mul(rel_pos, inv_ts))
            w_len = math.sqrt(_abs_sq(w))
            unit_w = _mul(w, 1.0 / w_len) if w_len > EPS else (1.0, 0.0)
            direction = (unit_w[1], -unit_w[0])
            u = _mul(unit_w, comb_r * inv_ts - w_len)
        lines.append(Line(_add(vel, _mul(u, nb.responsibility)), direction))

    fail, result = _lp2(lines, max_speed, pref_vel, False)
    if fail < len(lines):
        result = _lp3(lines, n_obst, fail, max_speed, result)
    return result
