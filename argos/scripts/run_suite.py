#!/usr/bin/env python3
"""Large-scale ARGoS suite: SwarmX vs stop-and-wait for growing fleets.

    python3 scripts/run_suite.py                       # default sizes, 3 seeds
    python3 scripts/run_suite.py --sizes 10:8:30 50:24:150 --seeds 5 --jobs 8

Each size is ROBOTS:AISLES:TASKS (the warehouse grows with the fleet so
robot density stays comparable). Writes results/argos_suite.json and .md.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ARGOS = os.path.abspath(os.path.join(HERE, ".."))


def run(job):
    robots, aisles, tasks, mode, seed, length = job
    gen = [sys.executable, os.path.join(HERE, "generate_experiment.py"), "--robots", str(robots), "--aisles", str(aisles),
           "--tasks", str(tasks), "--mode", mode, "--seed", str(seed), "--length", str(length)]
    path = subprocess.run(gen, check=True, capture_output=True, text=True).stdout.strip().splitlines()[-1]
    res = os.path.join(ARGOS, "results", os.path.basename(path).replace(".argos", ".json"))
    if os.path.exists(res):
        os.remove(res)
    p = subprocess.run(["argos3", "-z", "-c", path], check=False, capture_output=True, text=True, timeout=7200)
    if not os.path.exists(res):  # crashed: report it instead of aborting the suite
        print(f"FAILED {os.path.basename(path)} (exit {p.returncode}): {p.stderr[-300:]}", file=sys.stderr)
        return {"mode": mode, "robots": robots, "tasks": tasks, "delivered": 0, "makespan_s": float(length),
                "total_completion_s": float(length) * tasks, "collisions": 0, "min_separation_m": 0.0,
                "seed": seed, "crashed": True}
    with open(res) as f:
        return json.load(f)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sizes", nargs="+", default=["10:8:30", "30:16:90", "50:24:150"])
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--modes", nargs="+", default=["swarmx", "stopwait"])
    ap.add_argument("--length", type=int, default=3000)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args()
    jobs = [(int(r), int(a), int(t), m, s, args.length) for r, a, t in (x.split(":") for x in args.sizes)
            for m in args.modes for s in range(1, args.seeds + 1)]
    with ThreadPoolExecutor(args.jobs) as ex:
        runs = list(ex.map(run, jobs))
    rows = []
    for r, a, t in (x.split(":") for x in args.sizes):
        row = {"robots": int(r), "aisles": int(a), "tasks": int(t)}
        for m in args.modes:
            rs = [x for x in runs if x["robots"] == int(r) and x["mode"] == m]
            row[m] = {k: statistics.fmean(x[k] for x in rs) for k in ("makespan_s", "total_completion_s", "delivered",
                                                                       "collisions", "min_separation_m")}
            row[m]["all_done"] = all(x["delivered"] == x["tasks"] for x in rs)
        if "swarmx" in row and "stopwait" in row:
            b, p = row["stopwait"]["total_completion_s"], row["swarmx"]["total_completion_s"]
            row["reduction_pct"] = round(100 * (b - p) / b, 1)
        rows.append(row)
    os.makedirs(os.path.join(ARGOS, "results"), exist_ok=True)
    with open(os.path.join(ARGOS, "results", "argos_suite.json"), "w") as f:
        json.dump({"rows": rows, "runs": runs}, f, indent=1)
    lines = ["| robots | aisles | tasks | mode | delivered | makespan [s] | total completion [s] | collisions | closest pass [m] |",
             "|---:|---:|---:|---|---:|---:|---:|---:|---:|"]
    for row in rows:
        for m in args.modes:
            d = row[m]
            lines.append(f"| {row['robots']} | {row['aisles']} | {row['tasks']} | {m} | {d['delivered']:.1f}/{row['tasks']} "
                         f"| {d['makespan_s']:.0f} | {d['total_completion_s']:.0f} | {d['collisions']:.1f} | {d['min_separation_m']:.3f} |")
        if "reduction_pct" in row:
            lines.append(f"| | | | **SwarmX reduction** | | | **{row['reduction_pct']}%** | | |")
    md = "\n".join(lines)
    with open(os.path.join(ARGOS, "results", "argos_suite.md"), "w") as f:
        f.write(md + "\n")
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
