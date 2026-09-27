"""Transports for the decentralized network stack.

All transports expose the same tiny interface::

    publish(msg: dict) -> None
    poll() -> list[dict]      # non-blocking, returns everything received
    close() -> None

* :class:`InProcBus` - simulated radio for the lightweight simulator:
  limited range, latency + jitter, packet loss, per-link statistics.
* :class:`UdpMulticastTransport` - zero-dependency LAN broadcast for real
  edge hardware (Raspberry Pi) without ROS.
* :class:`ZenohTransport` - Eclipse Zenoh peer mode (multicast scouting, no
  router), the same data plane rmw_zenoh uses under ROS 2.
* The ROS 2 transport lives in ``swarmx_fleet`` (topics over rmw_zenoh).
"""
from __future__ import annotations

import heapq
import itertools
import math
import random
import socket
import struct
import threading
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

from . import protocol


class Transport:
    def publish(self, msg: dict) -> None:
        raise NotImplementedError

    def poll(self) -> List[dict]:
        raise NotImplementedError

    def close(self) -> None:
        pass


# --------------------------------------------------------------------------- sim
class InProcBus:
    """Shared simulated wireless medium.

    ``position_of(node_id)`` gives the sender/receiver positions used for the
    range check; nodes with ``infinite_range`` (e.g. the dashboard) hear all.
    """

    def __init__(self, comm_range: float = 20.0, latency: float = 0.02, jitter: float = 0.02,
                 loss: float = 0.02, seed: int = 0):
        self.comm_range = comm_range
        self.latency = latency
        self.jitter = jitter
        self.loss = loss
        self.rng = random.Random(seed)
        self.now = 0.0
        self.endpoints: Dict[str, "BusEndpoint"] = {}
        self._queue: List[Tuple[float, int, str, dict]] = []
        self._seq = itertools.count()
        self.position_of: Callable[[str], Optional[Tuple[float, float]]] = lambda _id: None
        self.stats = {"sent": 0, "delivered": 0, "dropped": 0, "bytes": 0}
        self.down: set = set()  # node ids whose radio is off (simulated failure)

    def endpoint(self, node_id: str, infinite_range: bool = False) -> "BusEndpoint":
        ep = BusEndpoint(self, node_id, infinite_range)
        self.endpoints[node_id] = ep
        return ep

    def _send(self, src: str, msg: dict) -> None:
        if src in self.down:
            return
        size = len(protocol.encode(msg))
        self.stats["sent"] += 1
        self.stats["bytes"] += size
        sp = self.position_of(src)
        sender = self.endpoints.get(src)
        for nid, ep in self.endpoints.items():
            if nid == src or nid in self.down:
                continue
            if not (ep.infinite_range or (sender and sender.infinite_range)):
                rp = self.position_of(nid)
                if sp is not None and rp is not None and math.dist(sp, rp) > self.comm_range:
                    continue
            if self.rng.random() < self.loss:
                self.stats["dropped"] += 1
                continue
            due = self.now + self.latency + self.rng.random() * self.jitter
            heapq.heappush(self._queue, (due, next(self._seq), nid, msg))

    def advance(self, now: float) -> None:
        self.now = now
        q = self._queue
        while q and q[0][0] <= now:
            _, _, nid, msg = heapq.heappop(q)
            ep = self.endpoints.get(nid)
            if ep is not None:
                ep.inbox.append(msg)
                self.stats["delivered"] += 1


class BusEndpoint(Transport):
    def __init__(self, bus: InProcBus, node_id: str, infinite_range: bool):
        self.bus = bus
        self.node_id = node_id
        self.infinite_range = infinite_range
        self.inbox: Deque[dict] = deque()

    def publish(self, msg: dict) -> None:
        self.bus._send(self.node_id, msg)

    def poll(self) -> List[dict]:
        out = list(self.inbox)
        self.inbox.clear()
        return out


# --------------------------------------------------------------------------- UDP
class UdpMulticastTransport(Transport):
    """IPv4 multicast on the local link (TTL 1). No broker, no server."""

    def __init__(self, node_id: str, group: str = "239.255.77.1", port: int = 47077,
                 iface: str = "0.0.0.0", loopback: bool = True):
        self.node_id = node_id
        self.group, self.port = group, port
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.rx.bind(("", port))
        mreq = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton(iface))
        self.rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1 if loopback else 0)

    def publish(self, msg: dict) -> None:
        self.tx.sendto(protocol.encode(msg), (self.group, self.port))

    def poll(self) -> List[dict]:
        out = []
        while True:
            try:
                data, _ = self.rx.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            try:
                msg = protocol.decode(data)
            except ValueError:
                continue
            if msg.get("src") != self.node_id:
                out.append(msg)
        return out

    def close(self) -> None:
        self.rx.close()
        self.tx.close()


# ------------------------------------------------------------------------- Zenoh
class ZenohTransport(Transport):
    """Eclipse Zenoh in *peer* mode with multicast scouting (``pip install eclipse-zenoh``).

    Key expressions: ``swarmx/p2p/<type>/<src>``. Peers discover each other on
    the LAN and talk directly; a router is optional (only for bridging subnets).
    """

    def __init__(self, node_id: str, prefix: str = "swarmx/p2p", connect: Optional[List[str]] = None):
        import json as _json
        import zenoh  # optional dependency

        self.node_id = node_id
        self.prefix = prefix
        conf = zenoh.Config()
        conf.insert_json5("mode", '"peer"')
        conf.insert_json5("scouting/multicast/enabled", "true")
        if connect:
            conf.insert_json5("connect/endpoints", _json.dumps(connect))
        self.session = zenoh.open(conf)
        self._inbox: Deque[dict] = deque()
        self._lock = threading.Lock()

        def on_sample(sample):
            try:
                msg = protocol.decode(bytes(sample.payload.to_bytes()))
            except Exception:  # noqa: BLE001 - never let a bad packet kill the subscriber
                return
            if msg.get("src") != self.node_id:
                with self._lock:
                    self._inbox.append(msg)

        self._sub = self.session.declare_subscriber(f"{prefix}/**", on_sample)

    def publish(self, msg: dict) -> None:
        self.session.put(f"{self.prefix}/{msg['type']}/{self.node_id}", protocol.encode(msg))

    def poll(self) -> List[dict]:
        with self._lock:
            out = list(self._inbox)
            self._inbox.clear()
        return out

    def close(self) -> None:
        self._sub.undeclare()
        self.session.close()
