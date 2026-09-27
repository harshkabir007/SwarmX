import asyncio
import base64
import json
import os

from swarmx_core.dashboard.monitor import FleetMonitor
from swarmx_core.dashboard.server import DashboardServer
from swarmx_core.warehouse import Warehouse
from swarmx_core import protocol


def test_monitor_builds_snapshot_from_p2p():
    m = FleetMonitor()
    m.ingest(protocol.make(protocol.TASK, "wms", 0.0, tasks=[{"id": "T1", "pickup": [6, 5], "dropoff": [1, 4]}]), now=0.0)
    m.ingest(protocol.make(protocol.STATE, "robot1", 0.1, x=1, y=2, st="to_pickup", task="T1", bat=0.9, bundle=["T1"]), now=0.1)
    s = m.snapshot(now=0.2)
    assert s["robots"][0]["id"] == "robot1" and s["tasks"][0]["robot"] == "robot1"
    m.ingest(protocol.make(protocol.TASK_DONE, "robot1", 5.0, task="T1"), now=5.0)
    s = m.snapshot(now=5.1)
    assert s["metrics"]["delivered"] == 1 and s["tasks"] == []


def test_server_rest_and_websocket():
    wh = Warehouse()
    got = {}

    async def main():
        srv = DashboardServer(wh.to_dict, lambda: {"t": 1.0}, lambda c: {"ok": True, "echo": c["cmd"]},
                              host="127.0.0.1", port=0, info={"source": "test"})
        await srv.start()
        port = srv._server.sockets[0].getsockname()[1]
        r, w = await asyncio.open_connection("127.0.0.1", port)
        w.write(b"GET /api/world HTTP/1.1\r\nHost: x\r\n\r\n"); await w.drain()
        body = (await r.read()).split(b"\r\n\r\n", 1)[1]
        got["world"] = json.loads(body)
        r, w = await asyncio.open_connection("127.0.0.1", port)
        key = base64.b64encode(os.urandom(16)).decode()
        w.write(f"GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode())
        await w.drain()
        head = await r.readuntil(b"\r\n\r\n")
        got["status"] = head.split(b"\r\n")[0]
        b1, b2 = await r.readexactly(2)
        n = b2 & 0x7F
        if n == 126:
            n = int.from_bytes(await r.readexactly(2), "big")
        elif n == 127:
            n = int.from_bytes(await r.readexactly(8), "big")
        got["first"] = json.loads(await r.readexactly(n))
        # masked client frame with a command
        payload = json.dumps({"cmd": "pause"}).encode()
        mask = os.urandom(4)
        w.write(bytes([0x81, 0x80 | len(payload)]) + mask + bytes(p ^ mask[i % 4] for i, p in enumerate(payload)))
        await w.drain()
        while True:
            b1, b2 = await r.readexactly(2)
            n = b2 & 0x7F
            if n == 126:
                n = int.from_bytes(await r.readexactly(2), "big")
            msg = json.loads(await r.readexactly(n))
            if msg.get("type") == "cmd_result":
                got["result"] = msg
                break
        w.close()

    asyncio.run(asyncio.wait_for(main(), 10))
    assert got["world"]["width"] == wh.width and got["world"]["info"]["source"] == "test"
    assert b"101" in got["status"] and got["first"]["type"] == "world"
    assert got["result"]["echo"] == "pause"
