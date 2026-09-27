#!/usr/bin/env python3
"""Poll the SwarmX dashboard API until all tasks are delivered (or timeout); print a JSON verdict."""
import json
import sys
import time
import urllib.request

timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 600
expected = int(sys.argv[2]) if len(sys.argv) > 2 else 0
t0 = time.time()
last = None
while time.time() - t0 < timeout:
    time.sleep(10)
    try:
        d = json.load(urllib.request.urlopen("http://localhost:8080/api/snapshot", timeout=3))
    except Exception:  # noqa: BLE001 - dashboard not up yet
        continue
    m, sim = d["metrics"], d.get("sim") or {}
    last = {"wall_s": round(time.time() - t0), "sim_t": round(d["t"] - d.get("t_start", d["t"]), 1),
            "delivered": m["delivered"], "robots": m["robots_alive"], "blocked_cells": len(d["blocked"]),
            "collisions": sim.get("collisions"), "min_separation_m": sim.get("min_separation")}
    print(json.dumps(last), flush=True)
    if expected and m["delivered"] >= expected:
        break
ok = bool(last) and (not expected or last["delivered"] >= expected) and (last["collisions"] or 0) == 0
print("RESULT", "PASS" if ok else "FAIL", json.dumps(last))
sys.exit(0 if ok else 1)
