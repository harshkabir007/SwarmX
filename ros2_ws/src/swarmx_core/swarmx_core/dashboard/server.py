"""Zero-dependency dashboard server: static files + REST + WebSocket (RFC 6455).

Pure asyncio + stdlib so it runs anywhere Python runs (including a Raspberry
Pi without internet access).

Endpoints
---------
GET  /                 dashboard UI (static/)
GET  /api/world        warehouse layout (JSON)
GET  /api/snapshot     latest fleet snapshot (JSON)
POST /api/cmd          operator command (JSON body) -> JSON result
GET  /ws               WebSocket: server pushes {"type": "world"|"snapshot", ...};
                       client sends commands as JSON text frames
"""
from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import mimetypes
import os
import struct
from typing import Awaitable, Callable, Dict, Optional, Set, Union

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

CommandHandler = Callable[[dict], Union[dict, Awaitable[dict]]]


class WebSocket:
    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self.reader, self.writer = reader, writer
        self.closed = False
        self._send_lock = asyncio.Lock()

    async def send_text(self, text: str) -> None:
        if self.closed:
            return
        data = text.encode()
        n = len(data)
        if n < 126:
            header = struct.pack("!BB", 0x81, n)
        elif n < 65536:
            header = struct.pack("!BBH", 0x81, 126, n)
        else:
            header = struct.pack("!BBQ", 0x81, 127, n)
        async with self._send_lock:
            try:
                self.writer.write(header + data)
                await self.writer.drain()
            except (ConnectionError, RuntimeError):
                self.closed = True

    async def _send_frame(self, opcode: int, payload: bytes = b"") -> None:
        async with self._send_lock:
            try:
                self.writer.write(struct.pack("!BB", 0x80 | opcode, len(payload)) + payload)
                await self.writer.drain()
            except (ConnectionError, RuntimeError):
                self.closed = True

    async def recv_text(self) -> Optional[str]:
        """Next text message, or None when the peer closes."""
        buf = b""
        while True:
            try:
                b1, b2 = await self.reader.readexactly(2)
            except (asyncio.IncompleteReadError, ConnectionError):
                self.closed = True
                return None
            fin, opcode = b1 & 0x80, b1 & 0x0F
            masked, n = b2 & 0x80, b2 & 0x7F
            if n == 126:
                n = struct.unpack("!H", await self.reader.readexactly(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", await self.reader.readexactly(8))[0]
            if n > 1 << 20:
                self.closed = True
                return None
            mask = await self.reader.readexactly(4) if masked else b"\0\0\0\0"
            payload = bytearray(await self.reader.readexactly(n))
            for i in range(n):
                payload[i] ^= mask[i % 4]
            if opcode == 0x8:  # close
                await self._send_frame(0x8)
                self.closed = True
                return None
            if opcode == 0x9:  # ping
                await self._send_frame(0xA, bytes(payload[:125]))
                continue
            if opcode == 0xA:
                continue
            buf += bytes(payload)
            if fin:
                return buf.decode(errors="replace")

    async def close(self) -> None:
        if not self.closed:
            await self._send_frame(0x8)
        self.closed = True
        try:
            self.writer.close()
        except RuntimeError:
            pass


class DashboardServer:
    def __init__(self, get_world: Callable[[], dict], get_snapshot: Callable[[], dict],
                 on_command: Optional[CommandHandler] = None, host: str = "0.0.0.0", port: int = 8080,
                 push_hz: float = 10.0, static_dir: str = STATIC_DIR, info: Optional[dict] = None):
        self.get_world = get_world
        self.get_snapshot = get_snapshot
        self.on_command = on_command
        self.host, self.port = host, port
        self.push_period = 1.0 / push_hz
        self.static_dir = static_dir
        self.info = info or {}
        self.clients: Set[WebSocket] = set()
        self._server: Optional[asyncio.base_events.Server] = None

    # ---------------------------------------------------------- lifecycle
    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle, self.host, self.port)
        asyncio.get_running_loop().create_task(self._pusher())

    async def serve_forever(self) -> None:
        await self.start()
        async with self._server:
            await self._server.serve_forever()

    async def _pusher(self) -> None:
        while True:
            await asyncio.sleep(self.push_period)
            if not self.clients:
                continue
            try:
                text = json.dumps({"type": "snapshot", **self.get_snapshot()}, separators=(",", ":"))
            except Exception as exc:  # noqa: BLE001 - keep serving even if a snapshot fails
                text = json.dumps({"type": "error", "error": str(exc)})
            for ws in list(self.clients):
                if ws.closed:
                    self.clients.discard(ws)
                else:
                    await ws.send_text(text)

    # ------------------------------------------------------------- HTTP
    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            request = await reader.readuntil(b"\r\n\r\n")
        except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, ConnectionError):
            writer.close()
            return
        lines = request.decode(errors="replace").split("\r\n")
        try:
            method, target, _ = lines[0].split(" ", 2)
        except ValueError:
            writer.close()
            return
        headers: Dict[str, str] = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        path = target.split("?", 1)[0]

        if path == "/ws" and headers.get("upgrade", "").lower() == "websocket":
            await self._websocket(reader, writer, headers)
            return
        try:
            if path == "/api/world" and method == "GET":
                await self._json(writer, {**self.get_world(), "info": self.info})
            elif path == "/api/snapshot" and method == "GET":
                await self._json(writer, self.get_snapshot())
            elif path == "/api/cmd" and method == "POST":
                n = int(headers.get("content-length", "0") or 0)
                body = await reader.readexactly(n) if n else b"{}"
                await self._json(writer, await self._command(json.loads(body or b"{}")))
            elif method == "GET":
                await self._static(writer, path)
            else:
                await self._respond(writer, 405, b"method not allowed", "text/plain")
        except (json.JSONDecodeError, ValueError) as exc:
            await self._json(writer, {"ok": False, "error": str(exc)}, status=400)
        except ConnectionError:
            pass
        finally:
            try:
                writer.close()
            except RuntimeError:
                pass

    async def _command(self, cmd: dict) -> dict:
        if not isinstance(cmd, dict) or "cmd" not in cmd:
            return {"ok": False, "error": "expected {\"cmd\": ...}"}
        if self.on_command is None:
            return {"ok": False, "error": "read-only dashboard"}
        res = self.on_command(cmd)
        if asyncio.iscoroutine(res):
            res = await res
        return res if isinstance(res, dict) else {"ok": True}

    async def _static(self, writer, path: str) -> None:
        rel = "index.html" if path in ("", "/") else path.lstrip("/")
        full = os.path.realpath(os.path.join(self.static_dir, rel))
        if not full.startswith(os.path.realpath(self.static_dir) + os.sep) or not os.path.isfile(full):
            await self._respond(writer, 404, b"not found", "text/plain")
            return
        with open(full, "rb") as f:
            data = f.read()
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        await self._respond(writer, 200, data, ctype)

    async def _json(self, writer, obj, status: int = 200) -> None:
        await self._respond(writer, status, json.dumps(obj, separators=(",", ":")).encode(), "application/json")

    @staticmethod
    async def _respond(writer, status: int, body: bytes, ctype: str) -> None:
        reason = {200: "OK", 400: "Bad Request", 404: "Not Found", 405: "Method Not Allowed"}.get(status, "OK")
        head = (f"HTTP/1.1 {status} {reason}\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n"
                "Cache-Control: no-store\r\nConnection: close\r\n\r\n").encode()
        writer.write(head + body)
        await writer.drain()

    # -------------------------------------------------------- WebSocket
    async def _websocket(self, reader, writer, headers: Dict[str, str]) -> None:
        key = headers.get("sec-websocket-key", "")
        accept = base64.b64encode(hashlib.sha1(key.encode() + _GUID).digest()).decode()
        writer.write(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                      f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
        await writer.drain()
        ws = WebSocket(reader, writer)
        await ws.send_text(json.dumps({"type": "world", **self.get_world(), "info": self.info}))
        self.clients.add(ws)
        try:
            while not ws.closed:
                text = await ws.recv_text()
                if text is None:
                    break
                try:
                    cmd = json.loads(text)
                except json.JSONDecodeError:
                    continue
                res = await self._command(cmd)
                await ws.send_text(json.dumps({"type": "cmd_result", "req": cmd.get("req"), **res}))
        finally:
            self.clients.discard(ws)
            await ws.close()
