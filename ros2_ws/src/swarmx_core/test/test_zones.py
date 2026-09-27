import itertools
import random

from swarmx_core.zones import HOLD, ZoneClaim, ZoneLocks, compatible


def c(mode, lo=0, hi=4, state=HOLD, phase="in", cur=-1):
    return ZoneClaim(mode=mode, key=0.0, lo=lo, hi=hi, state=state, phase=phase, cur=cur)


def test_convoy_same_direction_compatible():
    assert compatible(c("SN"), c("SN"), True)
    assert not compatible(c("SN"), c("NS"), True)


def test_exclusive_never_shares():
    assert not compatible(c("XX"), c("SN"), True)
    assert not compatible(c("SN"), c("XX"), True)


def test_disjoint_extents_share():
    assert compatible(c("SS", 0, 1), c("NN", 3, 4), True)
    assert not compatible(c("SS", 0, 2), c("NN", 3, 4), True)   # no free cell between


def test_stack_requires_depth_margin_and_not_leaving():
    me = c("SS", 0, 1)
    assert compatible(me, c("SS", 0, 4, phase="svc", cur=4), True)
    assert not compatible(me, c("SS", 0, 4, phase="svc", cur=2), True)
    assert not compatible(me, c("SS", 0, 4, phase="out", cur=4), True)
    assert not compatible(me, c("SS", 0, 4, phase="svc", cur=4), False)  # a claimant must enter first


def _wire(z: ZoneLocks):
    return z.wire()


def test_protocol_never_grants_incompatible_holds():
    """Randomised two-to-four robot races with message delay: incompatible holds never coexist."""
    rng = random.Random(7)
    modes = ["SN", "NS", "SS", "NN", "XX"]
    for trial in range(400):
        n = rng.randint(2, 4)
        locks = [ZoneLocks(f"r{i}", settle=0.3, zone_len={"Z": 5}) for i in range(n)]
        want = []
        for i in range(n):
            m = rng.choice(modes)
            lo, hi = (0, rng.randint(0, 4)) if m == "SS" else ((rng.randint(0, 4), 4) if m == "NN" else (0, 4))
            want.append((m, lo, hi, rng.uniform(0, 1.0)))
        inbox = []  # (deliver_t, receiver, sender, wire)
        t = 0.0
        for step in range(60):
            t += 0.05
            for i, lk in enumerate(locks):
                m, lo, hi, t_start = want[i]
                if t >= t_start and "Z" not in lk.mine:
                    lk.claim("Z", m, t, key=t_start + 1.0, lo=lo, hi=hi)
                if "Z" in lk.mine and lk.can_enter("Z", t):
                    lk.hold("Z")
                    lk.update("Z", cur=lo if m != "NN" else hi, phase="in")
            for i, lk in enumerate(locks):
                for j in range(n):
                    if j != i:
                        inbox.append((t + rng.uniform(0.02, 0.12), j, lk.me, _wire(lk)))
            due = [x for x in inbox if x[0] <= t]
            inbox = [x for x in inbox if x[0] > t]
            for _, j, src, w in due:
                locks[j].update_peer(src, w, t)
            holders = [(i, locks[i].mine["Z"]) for i in range(n) if locks[i].holds("Z")]
            for (i, a), (j, b) in itertools.combinations(holders, 2):
                ok = compatible(a, b, True) or compatible(b, a, True)
                assert ok, f"trial {trial}: {a} and {b} both hold Z"


def test_lane_safe_stack_rules():
    from swarmx_core.zones import lane_safe
    # shallow robot servicing near the north end, deep robot leaving behind it: safe (LIFO)
    assert lane_safe(c("NN", 4, 4, phase="svc", cur=4), c("NN", 2, 4, phase="out", cur=2))
    # both leaving the same way: safe
    assert lane_safe(c("NN", 4, 4, phase="out", cur=4), c("NN", 2, 4, phase="out", cur=4))
    # newcomer still heading deeper than a robot that is already coming out: unsafe
    assert not lane_safe(c("SS", 0, 3, phase="in", cur=1), c("SS", 0, 4, phase="out", cur=2))
    # opposite pass-throughs are never safe together
    assert not lane_safe(c("SN", phase="in", cur=1), c("NS", phase="in", cur=3))
