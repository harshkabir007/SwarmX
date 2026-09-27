"""Benchmark scenarios.

All scenarios are deterministic given ``seed``: the same tasks, start poses
and events are generated for every coordination method, so results are
directly comparable.
"""
from __future__ import annotations

import random
from typing import Callable, Dict, Optional

from ..warehouse import Warehouse
from .simulator import SimConfig, Simulation

METHODS: Dict[str, dict] = {
    # the proposed system
    "swarmx": dict(motion="swarmx", allocator="cbba"),
    # traditional baseline: block/cell reservation, stop-and-wait, one-way lanes, greedy claiming
    "stopwait": dict(motion="stopwait", allocator="greedy"),
    # ablations
    "swarmx_greedy": dict(motion="swarmx", allocator="greedy"),
    "stopwait_cbba": dict(motion="stopwait", allocator="cbba"),
    "swarmx_no_intent": dict(motion="swarmx", allocator="cbba", overrides={"congestion_routing": False, "reroute": False}),
    "swarmx_4conn": dict(motion="swarmx", allocator="cbba", overrides={"diagonal": False}),
    # interference-free lower bound (robots pass through each other; not physically realisable)
    "ghost": dict(motion="swarmx", allocator="cbba", overrides={"ghost": True}),
}


def _crossing(sim: Simulation, rng: random.Random, n_tasks: int) -> None:
    """Overlapping paths: shuttles between the south and north cross aisles.

    Every task's shortest path runs through one of four shared single-lane
    aisle columns, alternating direction, so robots meet head-on at the
    choke points - the case the challenge's 20% criterion is about.
    """
    wh = sim.wh
    cols = wh.aisle_xs[2:6]
    for i in range(n_tasks):
        x = rng.choice(cols)
        south, north = (x, rng.choice((2, 3))), (x, rng.choice((16, 17)))
        a, b = (south, north) if i % 2 == 0 else (north, south)
        sim.wms.new_task(0.0, a, b)


def _hot(sim: Simulation, rng: random.Random, n_tasks: int) -> None:
    """Fast-moving SKUs concentrated in three central aisles."""
    wh = sim.wh
    picks = [c for c in wh.pickups if c[0] in wh.aisle_xs[2:5]]
    for _ in range(n_tasks):
        sim.wms.new_task(0.0, rng.choice(picks), rng.choice(wh.dropoffs))


def _random(sim: Simulation, rng: random.Random, n_tasks: int) -> None:
    wh = sim.wh
    for _ in range(n_tasks):
        sim.wms.new_task(0.0, rng.choice(wh.pickups), rng.choice(wh.dropoffs))


def _blocked(sim: Simulation, rng: random.Random, n_tasks: int) -> None:
    """Random picks; pallets block two aisles mid-run (one is a pick face) and are cleared later."""
    _random(sim, rng, n_tasks)
    sim.events += [(20.0, "block", [(12, 6), (18, 13)]), (140.0, "unblock", [(12, 6)]), (200.0, "unblock", [(18, 13)])]
    sim.events.sort(key=lambda e: e[0])


def _failure(sim: Simulation, rng: random.Random, n_tasks: int) -> None:
    """Random picks; robot2 dies (motion + radio) at t=30 s and reboots at t=150 s.

    While it is down its tasks must be re-allocated by the survivors, and its
    body becomes an obstacle the others must perceive and avoid."""
    _random(sim, rng, n_tasks)
    sim.events += [(30.0, "fail", "robot2"), (150.0, "recover", "robot2")]
    sim.events.sort(key=lambda e: e[0])


SCENARIOS: Dict[str, Callable] = {
    "crossing": _crossing,
    "hot_aisles": _hot,
    "random": _random,
    "blocked_aisle": _blocked,
    "robot_failure": _failure,
}


def default_tasks(scenario: str, n_robots: int) -> int:
    if scenario == "crossing":
        return 4 * n_robots
    return max(20, 4 * n_robots)


def build(scenario: str, n_robots: int, seed: int, method: str = "swarmx",
          n_tasks: Optional[int] = None, max_time: float = 1500.0) -> Simulation:
    m = METHODS[method]
    cfg = SimConfig(n_robots=n_robots, n_tasks=0, seed=seed, motion=m["motion"], allocator=m["allocator"],
                    max_time=max_time, agent_overrides=dict(m.get("overrides", {})))
    sim = Simulation(cfg, Warehouse())
    rng = random.Random(1000 + seed)
    SCENARIOS[scenario](sim, rng, n_tasks if n_tasks is not None else default_tasks(scenario, n_robots))
    return sim
