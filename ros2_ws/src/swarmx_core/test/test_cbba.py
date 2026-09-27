import math
import random

from swarmx_core.cbba import CBBAAgent


def make_agents(n, pos, rng):
    agents = []
    for i in range(n):
        start = pos[i]
        a = CBBAAgent(f"r{i}", lambda p, q: math.dist(p, q), service_time=1.0, max_bundle=3)
        a.start = start
        agents.append(a)
    return agents


def run_consensus(agents, rounds=60, drop=0.0, rng=None, t0=0.0):
    t = t0
    for _ in range(rounds):
        t += 0.1
        for a in agents:
            a.build_bundle()
        msgs = [dict(src=a.id, **a.message_fields(t)) for a in agents]
        for a in agents:
            for m in msgs:
                if m["src"] != a.id and not (rng and rng.random() < drop):
                    a.consensus(m, t)
    for a in agents:
        a.build_bundle()
    return t


def tasks(m, rng):
    return [{"id": f"T{k:02d}", "pickup": [rng.randint(0, 30), rng.randint(0, 20)],
             "dropoff": [rng.randint(0, 30), rng.randint(0, 20)], "priority": 1.0} for k in range(m)]


def check_conflict_free(agents):
    owner = {}
    for a in agents:
        for tid in a.bundle:
            assert tid not in owner, f"{tid} in bundles of {owner.get(tid)} and {a.id}"
            owner[tid] = a.id
    # all agents agree on winners for every assigned task
    for tid, who in owner.items():
        for a in agents:
            assert a.z.get(tid) == who
    return owner


def test_converges_conflict_free():
    rng = random.Random(1)
    agents = make_agents(5, [(rng.randint(0, 30), rng.randint(0, 20)) for _ in range(5)], rng)
    for spec in tasks(12, rng):
        for a in agents:
            a.add_task(spec)
    run_consensus(agents)
    owner = check_conflict_free(agents)
    assert len(owner) == 12  # 5 robots x bundle 3 >= 12 tasks: everything allocated


def test_converges_with_packet_loss():
    rng = random.Random(2)
    agents = make_agents(4, [(0, 0), (30, 0), (0, 20), (30, 20)], rng)
    for spec in tasks(10, rng):
        agents[0].add_task(spec)  # only one robot knows the tasks: they must spread by gossip
    run_consensus(agents, rounds=120, drop=0.3, rng=rng)
    owner = check_conflict_free(agents)
    assert len(owner) == 10


def test_failed_agent_tasks_are_reallocated():
    rng = random.Random(3)
    agents = make_agents(3, [(0, 0), (15, 10), (30, 20)], rng)
    for spec in tasks(6, rng):
        for a in agents:
            a.add_task(spec)
    t = run_consensus(agents)
    dead = agents[1]
    lost = set(dead.bundle)
    assert lost, "precondition: middle robot won something"
    survivors = [agents[0], agents[2]]
    t += 10.0  # dead robot silent longer than agent_timeout
    for a in survivors:
        a.expire_agents(t)
    run_consensus(survivors, t0=t)
    owner = check_conflict_free(survivors)
    assert lost <= set(owner), "tasks of the failed robot were not re-allocated"


def test_locked_task_is_never_stolen():
    rng = random.Random(4)
    agents = make_agents(2, [(0, 0), (1, 1)], rng)
    spec = {"id": "T00", "pickup": [2, 2], "dropoff": [3, 3], "priority": 1.0}
    for a in agents:
        a.add_task(spec)
    agents[1].lock("T00")
    run_consensus(agents)
    assert agents[0].z["T00"] == "r1" and "T00" not in agents[0].bundle


def test_done_propagates():
    rng = random.Random(5)
    agents = make_agents(3, [(0, 0), (5, 5), (9, 9)], rng)
    for spec in tasks(4, rng):
        agents[0].add_task(spec)
    run_consensus(agents)
    agents[2].mark_done("T00", 1.0)
    run_consensus(agents)
    assert all("T00" not in a.tasks and "T00" in a.done for a in agents)
