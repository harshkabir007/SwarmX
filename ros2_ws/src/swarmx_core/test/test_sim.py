from swarmx_core.sim.scenarios import build


def _run(scenario, n, method, seed=1, max_time=900):
    sim = build(scenario, n, seed, method, max_time=max_time)
    return sim, sim.run()


def test_swarmx_crossing_zero_collisions_and_complete():
    for seed in (1, 2):
        _, r = _run("crossing", 5, "swarmx", seed)
        assert r["collisions"] == 0 and r["all_done"], r


def test_swarmx_random_zero_collisions_and_complete():
    _, r = _run("random", 4, "swarmx")
    assert r["collisions"] == 0 and r["all_done"], r


def test_baseline_is_collision_free_too():
    _, r = _run("random", 4, "stopwait")
    assert r["collisions"] == 0 and r["all_done"], r


def test_robot_failure_tasks_reallocated():
    sim, r = _run("robot_failure", 4, "swarmx")
    assert r["all_done"] and r["collisions"] == 0, r
    # while robot2 was down (t=30..150 s) the others kept delivering
    during = [t for t, when in sim.wms.done.items() if 40.0 < when < 150.0]
    assert during and all(sim.wms.done_by.get(t) != "robot2" for t in during)


def test_blocked_aisle_is_routed_around():
    sim = build("blocked_aisle", 4, 1, "swarmx", max_time=900)
    while sim.t < 100.0:
        sim.step()
    # every robot learned about the pallets through P2P broadcasts or its own sensing
    assert all({(12, 6), (18, 13)} <= rb.agent.planner.blocked for rb in sim.robots.values())
    r = sim.run()
    assert r["all_done"] and r["collisions"] == 0, r
    # once cleared, everybody forgets the obstacle again
    assert all((18, 13) not in rb.agent.planner.blocked for rb in sim.robots.values())
