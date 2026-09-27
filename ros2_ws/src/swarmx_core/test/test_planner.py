import math

from swarmx_core.planner import GridPlanner, path_length, zone_traversals
from swarmx_core.warehouse import Warehouse


def valid(wh, planner, path):
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        assert max(abs(dx), abs(dy)) == 1
        assert planner.passable(b)
        if dx and dy:
            assert planner.passable((a[0] + dx, a[1])) and planner.passable((a[0], a[1] + dy))


def test_astar_valid_and_optimal_vs_dijkstra():
    wh = Warehouse(); p = GridPlanner(wh)
    for s, g in [((2, 2), (27, 16)), ((6, 5), (1, 15)), ((31, 9), (9, 13))]:
        path = p.astar(s, g)
        assert path[0] == s and path[-1] == g
        valid(wh, p, path)
        assert math.isclose(path_length(path), p.distance(s, g), rel_tol=1e-9)


def test_blocked_cells_are_avoided():
    wh = Warehouse(); p = GridPlanner(wh)
    p.set_blocked([(12, 6)])
    path = p.astar((12, 3), (12, 9))
    assert (12, 6) not in path
    valid(wh, p, path)


def test_one_way_lanes_are_respected():
    wh = Warehouse(); p = GridPlanner(wh)
    p.lanes = wh.lanes(); p.allow_diagonal = False
    path = p.astar((10, 9), (20, 9))
    for a, b in zip(path, path[1:]):
        dx = b[0] - a[0]
        if dx:
            assert p.lanes.get(a, 0) * dx >= 0 and p.lanes.get(b, 0) * dx >= 0


def test_zone_traversal_modes():
    wh = Warehouse(); p = GridPlanner(wh)
    through = p.plan_via((12, 2), [(12, 17)])
    modes = [t["mode"] for t in zone_traversals(wh, through)]
    assert modes == ["SN", "SN"]
    pick = p.plan_via((12, 2), [(12, 6), (12, 2)])
    tr = zone_traversals(wh, pick)
    assert [t["mode"] for t in tr] == ["SS"] and tr[0]["hi"] == 2
