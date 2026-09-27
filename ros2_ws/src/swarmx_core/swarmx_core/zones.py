"""Decentralized zone locks for choke points (single-lane aisles, doors, lifts).

Each robot announces, inside its periodic ``state`` broadcast, the zones it
*claims* (will enter soon) and *holds* (is inside). No lock server exists:
every robot evaluates the same deterministic rule on what it hears.

A claim describes the traversal precisely:

* ``mode``  entry+exit end: ``SN``/``NS`` pass-through, ``SS``/``NN`` enter,
  service a pick face, and back out; ``XX`` exclusive (unknown extent)
* ``lo, hi`` the zone-local cell interval the robot will ever occupy
  (index 0 is the S end)
* ``key``   precedence = expected entry time, fixed when the claim is made
* ``phase`` ``in`` (heading to its deepest point), ``svc`` (servicing), ``out``
* ``cur``   current zone-local index (-1 while outside)

I may enter zone Z iff my claim has been visible for ``settle`` seconds and
every peer that holds Z, or claims Z with precedence over me, is *compatible*:

1. **disjoint extents** (one free cell in between) - robots can never meet;
2. **convoy** - same pass-through direction, the lane streams one way;
3. **stack** - same entry end, peer already inside, not leaving, and at
   least two cells deeper than my deepest cell. The later robot always exits
   first (LIFO), which pipelines pick service times without head-on meetings.

Deadlock freedom: precedence is a total order fixed at claim time; robots wait
only *outside* zones; inside a zone every robot's remaining motion is either
away from all robots it could meet (disjoint / convoy) or toward the exit with
every robot between it and the exit also heading out (stack + exit guard).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

CLAIM, HOLD = "claim", "hold"
PASS_MODES = ("SN", "NS")
STACK_MODES = ("SS", "NN")


@dataclass
class ZoneClaim:
    mode: str
    key: float
    lo: int = 0
    hi: int = 1 << 20
    state: str = CLAIM
    eta: float = 0.0     # seconds until I expect to be out of the zone
    phase: str = "in"
    cur: int = -1
    t_claim: float = 0.0

    def wire(self, zid: str) -> list:
        return [zid, self.mode, round(self.key, 3), self.state, round(self.eta, 2),
                self.lo, self.hi, self.phase, self.cur]


def parse(entry: list) -> Tuple[str, ZoneClaim]:
    """Wire list -> (zone id, claim). Tolerates the short legacy form."""
    zid, mode, key, state = entry[0], entry[1], entry[2], entry[3]
    c = ZoneClaim(mode=mode, key=key, state=state)
    if len(entry) > 4:
        c.eta = entry[4]
    if len(entry) > 8:
        c.lo, c.hi, c.phase, c.cur = entry[5], entry[6], entry[7], entry[8]
    return zid, c


def compatible(me: ZoneClaim, other: ZoneClaim, other_holds: bool) -> bool:
    if me.mode == "XX" or other.mode == "XX":
        return False
    if me.hi + 1 < other.lo or other.hi + 1 < me.lo:
        return True                                   # disjoint extents
    if me.mode == other.mode and me.mode in PASS_MODES:
        return True                                   # convoy
    if me.mode == other.mode and me.mode in STACK_MODES and other_holds:
        if other.phase == "out" or other.cur < 0:
            return False
        if me.mode == "SS":
            return me.hi + 2 <= other.cur and other.hi >= other.cur
        return me.lo - 2 >= other.cur and other.lo <= other.cur
    return False


def lane_safe(a: ZoneClaim, b: ZoneClaim) -> bool:
    """Physical check for two robots both inside one single-lane zone.

    Safe iff their remaining motions never require passing each other:
    disjoint extents, a same-direction convoy, or a LIFO stack in which the
    robot nearer the shared end never needs to go deeper than the other.
    """
    if a.mode == "XX" or b.mode == "XX":
        return False
    if a.hi + 1 < b.lo or b.hi + 1 < a.lo:
        return True
    if a.mode == b.mode and a.mode in PASS_MODES:
        return True
    if a.mode == b.mode and a.mode in STACK_MODES:
        if a.phase == "out" and b.phase == "out":
            return True  # both leaving through the shared end: a convoy out
        if a.mode == "SS":
            outer, inner = (a, b) if (a.cur if a.cur >= 0 else -1) <= (b.cur if b.cur >= 0 else -1) else (b, a)
            return inner.cur >= 0 and outer.hi < inner.cur
        big = 1 << 20
        outer, inner = (a, b) if (a.cur if a.cur >= 0 else big) >= (b.cur if b.cur >= 0 else big) else (b, a)
        return inner.cur >= 0 and outer.lo > inner.cur
    return False


class ZoneLocks:
    def __init__(self, me: str, settle: float = 0.3, stale_claim: float = 1.5, stale_hold: float = 10.0,
                 late_grace: float = 4.0, zone_len: Optional[Dict[str, int]] = None, fair_wait: float = 8.0):
        self.me = me
        self.settle = settle
        self.stale_claim = stale_claim
        self.stale_hold = stale_hold
        self.late_grace = late_grace
        self.zone_len = zone_len or {}
        self.fair_wait = fair_wait
        self.mine: Dict[str, ZoneClaim] = {}
        self.peers: Dict[str, Tuple[float, List[list]]] = {}

    # ------------------------------------------------------------ peers
    def update_peer(self, rid: str, zones: List[list], t_rx: float) -> None:
        self.peers[rid] = (t_rx, zones or [])

    def forget_peer(self, rid: str) -> None:
        self.peers.pop(rid, None)

    def _peer_entries(self, zone: str, now: float):
        for rid, (t_rx, zones) in self.peers.items():
            age = now - t_rx
            for entry in zones:
                if entry[0] == zone:
                    yield rid, age, parse(entry)[1]

    # ------------------------------------------------------------- mine
    def claim(self, zone: str, mode: str, now: float, key: Optional[float] = None,
              lo: int = 0, hi: int = 1 << 20, allow_rekey: bool = True) -> ZoneClaim:
        c = self.mine.get(zone)
        key = now if key is None else key
        if c is None:
            c = self.mine[zone] = ZoneClaim(mode, key, lo, hi, t_claim=now)
        elif c.state == CLAIM:
            if (c.mode, c.lo, c.hi) != (mode, lo, hi):
                c.mode, c.lo, c.hi = mode, lo, hi
            if allow_rekey and now > c.key + self.late_grace:
                # late because of our own travel (not queueing): re-queue instead of blocking others
                c.key, c.t_claim = key, now
        return c

    def cancel(self, zone: str) -> None:
        c = self.mine.get(zone)
        if c is not None and c.state == CLAIM:
            del self.mine[zone]

    def release(self, zone: str) -> None:
        self.mine.pop(zone, None)

    def hold(self, zone: str) -> None:
        if zone in self.mine:
            self.mine[zone].state = HOLD

    def holds(self, zone: str) -> bool:
        c = self.mine.get(zone)
        return c is not None and c.state == HOLD

    def update(self, zone: str, **fields) -> None:
        c = self.mine.get(zone)
        if c is not None:
            for k, v in fields.items():
                setattr(c, k, v)

    def set_eta(self, zone: str, eta: float) -> None:
        self.update(zone, eta=eta)

    # ---------------------------------------------------------- decision
    def blockers(self, zone: str, now: float) -> List[Tuple[str, str, float]]:
        """Peers that currently prevent me from entering ``zone`` -> ``(peer, state, eta)``."""
        mine = self.mine.get(zone)
        if mine is None:
            return [("<unclaimed>", CLAIM, 0.0)]
        if mine.state == HOLD:
            return []
        out = []
        if now - mine.t_claim < self.settle:
            out.append(("<settling>", CLAIM, self.settle - (now - mine.t_claim)))
        my_key = (mine.key, self.me)
        entries = list(self._peer_entries(zone, now))
        joinable = self._convoy_open(zone, mine, entries)
        for rid, age, other in entries:
            if other.state == HOLD and age < self.stale_hold:
                if not compatible(mine, other, True):
                    out.append((rid, HOLD, other.eta))
            elif other.state == CLAIM and age < self.stale_claim and (other.key, rid) < my_key:
                if compatible(mine, other, False):
                    continue
                if joinable and now - other.key < self.fair_wait:
                    continue  # bounded convoy batching: slip in behind a same-direction leader
                out.append((rid, CLAIM, other.eta))
        return out

    def _convoy_open(self, zone: str, mine: ZoneClaim, entries) -> bool:
        """A same-direction leader is inside and still >= 2 cells from the exit.

        Joining it is safe even with an older opposite claim queued: my hold is
        broadcast long before the leader can release, so the opposite robot
        always sees one of us holding.
        """
        if mine.mode not in PASS_MODES:
            return False
        n = self.zone_len.get(zone)
        if n is None:
            return False
        for rid, age, other in entries:
            if other.state != HOLD or other.mode != mine.mode or age > 0.5 or other.cur < 0:
                continue
            remaining = (n - 1 - other.cur) if mine.mode == "SN" else other.cur
            if remaining >= 2:
                return True
        return False

    def can_enter(self, zone: str, now: float) -> bool:
        return not self.blockers(zone, now)

    def stacked_conflict(self, zone: str, now: float) -> Optional[str]:
        """I am heading *in* but a peer deeper in my stack is already heading out (rare race)."""
        mine = self.mine.get(zone)
        if mine is None or mine.state != HOLD or mine.mode not in STACK_MODES or mine.phase != "in":
            return None
        for rid, age, other in self._peer_entries(zone, now):
            if age > self.stale_claim or other.state != HOLD or other.mode != mine.mode or other.phase != "out":
                continue
            if not lane_safe(mine, other):
                return rid
        return None

    def exit_blocked_by_stacker(self, zone: str, now: float) -> bool:
        """Exit guard: a robot stacked between me and the exit is still heading in."""
        mine = self.mine.get(zone)
        if mine is None or mine.mode not in STACK_MODES or mine.cur < 0:
            return False
        for rid, age, other in self._peer_entries(zone, now):
            if age > self.stale_claim or other.state != HOLD or other.mode != mine.mode or other.phase != "in":
                continue
            between = other.cur < mine.cur if mine.mode == "SS" else other.cur > mine.cur
            if between or other.cur < 0:
                return True
        return False

    def peer_load(self, now: float) -> Dict[str, float]:
        """Rough seconds of occupancy announced by peers per zone (for routing)."""
        load: Dict[str, float] = {}
        for rid, (t_rx, zones) in self.peers.items():
            if now - t_rx > self.stale_claim:
                continue
            for entry in zones:
                eta = entry[4] if len(entry) > 4 else 3.0
                load[entry[0]] = load.get(entry[0], 0.0) + max(eta, 1.0)
        return load

    def wire(self) -> List[list]:
        return [c.wire(zid) for zid, c in self.mine.items()]
