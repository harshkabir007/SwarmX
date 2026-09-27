import math

from swarmx_core.warehouse import Warehouse
from swarmx_fleet.perception import ScanPerception


def _scan_towards(pose, target, n=360, rmax=12.0, radius=0.3):
    """Synthetic scan: one disc obstacle at ``target``; everything else out of range."""
    x0, y0, th = pose
    ranges = []
    inc = 2 * math.pi / n
    for i in range(n):
        a = th - math.pi + i * inc
        dx, dy = math.cos(a), math.sin(a)
        fx, fy = target[0] - x0, target[1] - y0
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - c
        ranges.append(b - math.sqrt(disc) if disc >= 0 and b > 0 else float("inf"))
    return ranges, -math.pi, inc


def test_obstacle_in_free_space_is_confirmed_after_hysteresis():
    wh = Warehouse()
    p = ScanPerception(wh, confirm=3)
    pose = (12.5, 2.5, 0.0)            # bottom cross aisle
    ranges, amin, inc = _scan_towards(pose, (15.5, 2.5))
    for k in range(3):
        blocked, free, bodies = p.process(pose, ranges, amin, inc, 0.3, 12.0)
    assert (15, 2) in blocked
    assert bodies, "a compact unknown body should be reported for ORCA"


def test_peer_robot_is_not_reported_as_obstacle():
    wh = Warehouse()
    p = ScanPerception(wh, confirm=1)
    pose = (12.5, 2.5, 0.0)
    ranges, amin, inc = _scan_towards(pose, (15.5, 2.5))
    blocked, free, bodies = p.process(pose, ranges, amin, inc, 0.3, 12.0, peers=[(15.5, 2.5)])
    assert (15, 2) not in blocked and not bodies


def test_obstacle_clears_when_seen_free():
    wh = Warehouse()
    p = ScanPerception(wh, confirm=1, clear=2)
    pose = (12.5, 2.5, 0.0)
    ranges, amin, inc = _scan_towards(pose, (15.5, 2.5))
    p.process(pose, ranges, amin, inc, 0.3, 12.0)
    empty = [float("inf")] * len(ranges)
    for _ in range(2):
        blocked, free, _ = p.process(pose, empty, amin, inc, 0.3, 12.0)
    assert (15, 2) not in blocked and (15, 2) in free
