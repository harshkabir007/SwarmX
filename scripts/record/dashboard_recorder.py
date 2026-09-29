#!/usr/bin/env python3
"""Record the SwarmX dashboard as JPEG frames using headless Chrome (DevTools protocol).

Pure standard library: launches Chrome with --remote-debugging-port, talks CDP
over a minimal WebSocket client and captures a screenshot every --interval
seconds until every task is delivered (plus --tail seconds) or --timeout.
Encode afterwards, e.g. ffmpeg -framerate 10 -i frames/%05d.jpg out.mp4.

    python3 dashboard_recorder.py --frames /tmp/dash_frames --tasks 40
"""
import argparse
import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request


class CDP:
    def __init__(self, ws_url: str):
        host_port, path = ws_url[len("ws://"):].split("/", 1)
        host, port = host_port.split(":")
        self.sock = socket.create_connection((host, int(port)), timeout=30)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((f"GET /{path} HTTP/1.1\r\nHost: {host_port}\r\nUpgrade: websocket\r\n"
                           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += self.sock.recv(4096)
        if b" 101 " not in buf.split(b"\r\n", 1)[0]:
            raise RuntimeError("websocket handshake failed")
        self.rest = buf.split(b"\r\n\r\n", 1)[1]
        self.next_id = 0

    def _read(self, n: int) -> bytes:
        while len(self.rest) < n:
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise ConnectionError("devtools closed")
            self.rest += chunk
        out, self.rest = self.rest[:n], self.rest[n:]
        return out

    def _recv_msg(self) -> dict:
        data = b""
        while True:
            b1, b2 = self._read(2)
            n = b2 & 0x7F
            if n == 126:
                n = struct.unpack("!H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", self._read(8))[0]
            data += self._read(n)
            if b1 & 0x80:
                return json.loads(data)

    def call(self, method: str, **params) -> dict:
        self.next_id += 1
        payload = json.dumps({"id": self.next_id, "method": method, "params": params}).encode()
        mask = os.urandom(4)
        n = len(payload)
        head = bytes([0x81]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack("!H", n))
        self.sock.sendall(head + mask + bytes(p ^ mask[i % 4] for i, p in enumerate(payload)))
        while True:
            msg = self._recv_msg()
            if msg.get("id") == self.next_id:
                if "error" in msg:
                    raise RuntimeError(msg["error"])
                return msg.get("result", {})


def find_chrome() -> str:
    for c in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
        if shutil.which(c):
            return shutil.which(c)
    raise SystemExit("Chrome/Chromium not found")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", required=True, help="output directory for JPEG frames")
    ap.add_argument("--tasks", type=int, required=True)
    ap.add_argument("--url", default="http://localhost:8080/")
    ap.add_argument("--width", type=int, default=1440)
    ap.add_argument("--height", type=int, default=1000)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--tail", type=float, default=8.0)
    ap.add_argument("--timeout", type=float, default=3600.0)
    ap.add_argument("--port", type=int, default=9333)
    args = ap.parse_args()
    os.makedirs(args.frames, exist_ok=True)
    api = args.url.rstrip("/") + "/api/snapshot"

    # wait for the dashboard to be up
    t0 = time.time()
    while time.time() - t0 < 600:
        try:
            urllib.request.urlopen(api, timeout=3).read()
            break
        except Exception:  # noqa: BLE001
            time.sleep(2)
    profile = tempfile.mkdtemp(prefix="swarmx_chrome_")
    chrome = subprocess.Popen([find_chrome(), "--headless=new", "--disable-gpu", "--no-sandbox", "--hide-scrollbars",
                               f"--remote-debugging-port={args.port}", f"--user-data-dir={profile}",
                               f"--window-size={args.width},{args.height}", args.url],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(60):
            try:
                pages = json.load(urllib.request.urlopen(f"http://127.0.0.1:{args.port}/json", timeout=2))
                ws = next(p["webSocketDebuggerUrl"] for p in pages if p.get("type") == "page")
                break
            except Exception:  # noqa: BLE001
                time.sleep(0.5)
        else:
            raise SystemExit("could not attach to Chrome")
        cdp = CDP(ws)
        cdp.call("Emulation.setDeviceMetricsOverride", width=args.width, height=args.height,
                 deviceScaleFactor=1, mobile=False)
        time.sleep(2.0)
        n, done_at, start = 0, None, time.time()
        while time.time() - start < args.timeout:
            tick = time.time()
            shot = cdp.call("Page.captureScreenshot", format="jpeg", quality=88)
            with open(os.path.join(args.frames, f"{n:05d}.jpg"), "wb") as f:
                f.write(base64.b64decode(shot["data"]))
            n += 1
            try:
                delivered = json.load(urllib.request.urlopen(api, timeout=2))["metrics"]["delivered"]
            except Exception:  # noqa: BLE001 - the simulation has shut down
                os.remove(os.path.join(args.frames, f"{n - 1:05d}.jpg"))  # this frame may already show "Disconnected"
                n -= 1
                print(f"dashboard gone after {n} frames - stopping", flush=True)
                break
            if delivered >= args.tasks and done_at is None:
                done_at = time.time()
            if n % 20 == 0:
                print(f"{n} frames, delivered {delivered}/{args.tasks}", flush=True)
            if done_at is not None and time.time() - done_at >= args.tail:
                break
            time.sleep(max(0.0, args.interval - (time.time() - tick)))
        print(json.dumps({"frames": n, "wall_s": round(time.time() - start, 1), "completed": done_at is not None}))
        return 0 if done_at is not None else 1
    finally:
        chrome.terminate()
        shutil.rmtree(profile, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
