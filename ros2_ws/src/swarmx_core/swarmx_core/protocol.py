"""SwarmX peer-to-peer wire protocol.

Every message is a flat JSON object with a ``type`` and a ``src`` (robot id).
There is no master: every robot broadcasts, every robot listens, and the
dashboard is just another (passive) peer.

Message types
-------------
state      10 Hz   pose, velocity, battery, status, short-horizon intent
                   (next waypoints) and zone claims/holds
cbba       ~2 Hz   CBBA consensus vectors: winning bids, winners, timestamps,
                   plus the known task table (gossiped so tasks spread multi-hop)
task       event   a new transport task (pickup -> drop-off) enters the swarm
task_done  event   task delivered, everybody drops it
blocked    event   cells observed blocked/cleared (e.g. pallet in an aisle)
cmd        event   operator command from the dashboard (addressed or broadcast)
"""
from __future__ import annotations

import json
from typing import Any, Dict

PROTOCOL_VERSION = 1

STATE = "state"
CBBA = "cbba"
TASK = "task"
TASK_DONE = "task_done"
BLOCKED = "blocked"
CMD = "cmd"

ALL_TYPES = (STATE, CBBA, TASK, TASK_DONE, BLOCKED, CMD)


def make(msg_type: str, src: str, t: float, **fields: Any) -> Dict[str, Any]:
    msg = {"v": PROTOCOL_VERSION, "type": msg_type, "src": src, "t": round(t, 4)}
    msg.update(fields)
    return msg


def encode(msg: Dict[str, Any]) -> bytes:
    return json.dumps(msg, separators=(",", ":")).encode()


def decode(data: bytes) -> Dict[str, Any]:
    msg = json.loads(data)
    if not isinstance(msg, dict) or "type" not in msg or "src" not in msg:
        raise ValueError("not a SwarmX message")
    return msg
