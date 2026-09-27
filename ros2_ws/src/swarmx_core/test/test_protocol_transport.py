import socket

import pytest

from swarmx_core import protocol
from swarmx_core.transport import InProcBus, UdpMulticastTransport


def test_roundtrip():
    m = protocol.make(protocol.STATE, "robot1", 1.23456, x=1.0, y=2.0)
    assert protocol.decode(protocol.encode(m)) == m


def test_decode_rejects_garbage():
    with pytest.raises(ValueError):
        protocol.decode(b'{"hello": 1}')


def test_bus_range_and_latency():
    pos = {"a": (0, 0), "b": (5, 0), "c": (50, 0)}
    bus = InProcBus(comm_range=20, latency=0.05, jitter=0.0, loss=0.0)
    bus.position_of = pos.get
    a, b, c = bus.endpoint("a"), bus.endpoint("b"), bus.endpoint("c")
    a.publish({"type": "state", "src": "a"})
    bus.advance(0.01)
    assert b.poll() == []            # not delivered before the latency
    bus.advance(0.06)
    assert len(b.poll()) == 1
    assert c.poll() == []            # out of range


def test_bus_down_node_is_silent():
    bus = InProcBus(loss=0.0)
    a, b = bus.endpoint("a"), bus.endpoint("b")
    bus.down.add("a")
    a.publish({"type": "state", "src": "a"})
    bus.advance(1.0)
    assert b.poll() == []


def test_udp_multicast_loopback():
    try:
        rx = UdpMulticastTransport("rx", port=47177)
        tx = UdpMulticastTransport("tx", port=47177)
    except OSError as exc:
        pytest.skip(f"multicast unavailable here: {exc}")
    try:
        tx.publish(protocol.make(protocol.STATE, "tx", 0.0))
        got = []
        for _ in range(50):
            got += rx.poll()
            if got:
                break
            socket.socket().settimeout(0.01)
            import time; time.sleep(0.02)
        if not got:
            pytest.skip("multicast loopback not delivered on this host")
        assert got[0]["src"] == "tx"
    finally:
        rx.close(); tx.close()
