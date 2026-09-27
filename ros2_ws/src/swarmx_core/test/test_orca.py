import math
import random

from swarmx_core import orca


def _sim(agents, steps=300, dt=0.05, radius=0.3, vmax=1.0, walls=()):
    """agents: list of [pos, vel, goal]; returns min pairwise distance."""
    min_d = math.inf
    for _ in range(steps):
        new = []
        for i, (p, v, g) in enumerate(agents):
            d = math.hypot(g[0] - p[0], g[1] - p[1])
            pref = ((g[0] - p[0]) / d * vmax, (g[1] - p[1]) / d * vmax) if d > 0.05 else (0.0, 0.0)
            nbrs = [orca.Neighbor(q, w, radius) for j, (q, w, _) in enumerate(agents) if j != i]
            new.append(orca.compute_velocity(p, v, pref, radius, vmax, nbrs, walls, 2.0, 0.5, dt))
        for a, v in zip(agents, new):
            a[1] = v
            a[0] = (a[0][0] + v[0] * dt, a[0][1] + v[1] * dt)
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                min_d = min(min_d, math.dist(agents[i][0], agents[j][0]))
    return min_d


def test_free_space_returns_preferred_velocity():
    v = orca.compute_velocity((0, 0), (0, 0), (0.5, 0.2), 0.3, 1.0, [])
    assert math.isclose(v[0], 0.5) and math.isclose(v[1], 0.2)


def test_speed_is_clamped():
    v = orca.compute_velocity((0, 0), (0, 0), (3.0, 4.0), 0.3, 1.0, [])
    assert math.isclose(math.hypot(*v), 1.0, rel_tol=1e-6)


def test_head_on_pair_never_collides():
    agents = [[(0.0, 0.0), (0.0, 0.0), (6.0, 0.0)], [(6.0, 0.0), (0.0, 0.0), (0.0, 0.0)]]
    assert _sim(agents) >= 0.6 - 1e-6


def test_circle_swap_eight_agents():
    agents = []
    for k in range(8):
        a = 2 * math.pi * k / 8
        p = (4 * math.cos(a), 4 * math.sin(a))
        agents.append([p, (0.0, 0.0), (-p[0], -p[1])])
    assert _sim(agents, steps=400) >= 0.6 - 0.02


def test_random_crowd_no_contacts():
    rng = random.Random(3)
    agents = [[(rng.uniform(0, 6), rng.uniform(0, 6)), (0.0, 0.0), (rng.uniform(0, 6), rng.uniform(0, 6))] for _ in range(10)]
    # separate initial overlaps
    for i in range(10):
        agents[i][0] = (i % 5 * 1.4, i // 5 * 3.0)
    assert _sim(agents, steps=300) >= 0.6 - 0.02


def test_wall_constraint_blocks_motion_into_wall():
    # wall face at x = 1.0, robot centre at 0.5 (radius 0.3 -> 0.2 m clearance)
    v = orca.compute_velocity((0.5, 0.0), (0.0, 0.0), (1.0, 0.0), 0.3, 1.0, [], [(1.0, 0.0)], 2.0, 0.5, 0.1)
    assert v[0] <= (1.0 - 0.5 - 0.3) / 0.5 + 1e-6
