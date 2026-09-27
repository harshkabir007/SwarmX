"""SwarmX benchmark: decentralized coordination vs traditional stop-and-wait.

Usage::

    python -m swarmx_core.sim.benchmark                      # default suite
    python -m swarmx_core.sim.benchmark --scenarios crossing --robots 3 5 8 --seeds 5
    python -m swarmx_core.sim.benchmark --methods swarmx stopwait ghost swarmx_no_intent

Writes ``results/benchmark.json`` and ``results/benchmark.md``.

Success criteria (from the problem statement):
  * zero inter-robot collisions (every SwarmX run),
  * >= 20 % reduction in total task completion time vs stop-and-wait on
    overlapping paths (the ``crossing`` scenario).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from multiprocessing import Pool
from typing import Dict, List

from .scenarios import METHODS, SCENARIOS, build

KEYS = ("makespan", "total_completion", "mean_completion", "collisions", "zone_violations", "zone_conflicts",
        "min_separation", "zone_wait",
        "stop_time", "reroutes", "deadlocks_resolved", "completed", "tasks", "msgs_per_robot_s", "kbps_per_robot")


def run_one(job) -> dict:
    scenario, n, seed, method, max_time = job
    t0 = time.time()
    sim = build(scenario, n, seed, method, max_time=max_time)
    r = sim.run()
    w = sim.wms
    comp = [w.done[tid] - w.tasks[tid]["created"] for tid in w.done if tid in w.tasks]
    # unfinished tasks count as finishing at the time limit (penalises gridlock honestly)
    unfinished = len(w.tasks) - len(comp)
    total = sum(comp) + unfinished * sim.t
    dur = max(sim.t, 1e-6)
    r.update({
        "scenario": scenario, "method": method, "n": n,
        "total_completion": round(total, 1),
        "msgs_per_robot_s": round(r["msgs"] / n / dur, 1),
        "kbps_per_robot": round(r["kbytes"] / n / dur, 2),
        "wall_s": round(time.time() - t0, 1),
    })
    return r


def _agg(vals: List[float]) -> Dict[str, float]:
    vals = [v for v in vals if v is not None]
    if not vals:
        return {"mean": None, "std": None}
    return {"mean": round(statistics.fmean(vals), 2), "std": round(statistics.pstdev(vals), 2) if len(vals) > 1 else 0.0}


def summarise(runs: List[dict], baseline: str = "stopwait", proposed: str = "swarmx") -> dict:
    groups: Dict[tuple, List[dict]] = {}
    for r in runs:
        groups.setdefault((r["scenario"], r["n"], r["method"]), []).append(r)
    table = []
    for (sc, n, m), rs in sorted(groups.items()):
        row = {"scenario": sc, "n": n, "method": m, "runs": len(rs),
               "all_done": all(x["all_done"] for x in rs)}
        for k in KEYS:
            row[k] = _agg([x.get(k) for x in rs])
        table.append(row)
    idx = {(r["scenario"], r["n"], r["method"]): r for r in table}
    comparisons = []
    for (sc, n, m), row in idx.items():
        if m != proposed or (sc, n, baseline) not in idx:
            continue
        b = idx[(sc, n, baseline)]
        c = {"scenario": sc, "n": n}
        for k in ("makespan", "total_completion", "mean_completion"):
            bm, pm = b[k]["mean"], row[k]["mean"]
            c[f"{k}_reduction_pct"] = round(100.0 * (bm - pm) / bm, 1) if bm and pm is not None else None
        c["baseline_all_done"] = b["all_done"]
        c["proposed_all_done"] = row["all_done"]
        comparisons.append(c)
    swarm_runs = [r for r in runs if r["method"] == proposed]
    crossing = [c for c in comparisons if c["scenario"] == "crossing"]
    verdict = {
        "zero_collisions": all(r["collisions"] == 0 for r in swarm_runs) if swarm_runs else None,
        "swarmx_collisions_total": sum(r["collisions"] for r in swarm_runs),
        "swarmx_zone_conflicts_total": sum(r.get("zone_conflicts", 0) for r in swarm_runs),
        "swarmx_zone_violations_total": sum(r.get("zone_violations", 0) for r in swarm_runs),
        "swarmx_runs": len(swarm_runs),
        "swarmx_all_tasks_completed": all(r["all_done"] for r in swarm_runs) if swarm_runs else None,
        "crossing_total_completion_reduction_pct": {c["n"]: c["total_completion_reduction_pct"] for c in crossing},
        "crossing_makespan_reduction_pct": {c["n"]: c["makespan_reduction_pct"] for c in crossing},
        "meets_20pct_on_overlapping_paths": (all((c["total_completion_reduction_pct"] or 0) >= 20 for c in crossing)
                                            if crossing else None),
    }
    return {"table": table, "comparisons": comparisons, "verdict": verdict}


def _fmt(a: dict, nd: int = 1) -> str:
    if a["mean"] is None:
        return "-"
    return f"{a['mean']:.{nd}f} ± {a['std']:.{nd}f}" if a["std"] else f"{a['mean']:.{nd}f}"


def to_markdown(summary: dict, meta: dict) -> str:
    out = ["# SwarmX benchmark", "",
           f"Generated {meta['date']} · seeds={meta['seeds']} · {meta['runs']} runs · {meta['wall_s']:.0f}s wall", "",
           "## Success criteria", ""]
    v = summary["verdict"]
    out += [f"* Zero inter-robot collisions: **{'PASS' if v['zero_collisions'] else 'FAIL'}** "
            f"({v['swarmx_collisions_total']} collisions in {v['swarmx_runs']} SwarmX runs)",
            f"* All tasks completed by SwarmX in every run: **{'yes' if v['swarmx_all_tasks_completed'] else 'NO'}**",
            f"* Aisle-lock safety (ground truth): {v['swarmx_zone_conflicts_total']} incompatible co-occupancies, "
            f"{v['swarmx_zone_violations_total']} entries without a lock"]
    if v["crossing_total_completion_reduction_pct"]:
        red = ", ".join(f"{n} robots: {p}%" for n, p in sorted(v["crossing_total_completion_reduction_pct"].items()))
        out.append(f"* Total task completion time reduction on overlapping paths (crossing) vs stop-and-wait: {red} "
                   f"-> **{'PASS' if v['meets_20pct_on_overlapping_paths'] else 'NOT MET for every fleet size'}** (target >= 20%)")
    out += ["", "## SwarmX vs stop-and-wait", "",
            "| scenario | robots | total completion time | makespan | mean task time | baseline finished all tasks |",
            "|---|---:|---:|---:|---:|:---:|"]
    for c in sorted(summary["comparisons"], key=lambda c: (c["scenario"], c["n"])):
        out.append(f"| {c['scenario']} | {c['n']} | {c['total_completion_reduction_pct']}% | {c['makespan_reduction_pct']}% "
                   f"| {c['mean_completion_reduction_pct']}% | {'yes' if c['baseline_all_done'] else '**no (gridlock)**'} |")
    out += ["", "Reductions are relative to the stop-and-wait baseline (positive = SwarmX faster). "
            "Unfinished tasks are charged the time limit.", "",
            "## All runs (mean ± std over seeds)", "",
            "| scenario | robots | method | done | total completion [s] | makespan [s] | collisions | min sep [m] | zone wait [s] | stop time [s] | reroutes | msgs/robot/s | kB/s/robot |",
            "|---|---:|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for r in summary["table"]:
        out.append(f"| {r['scenario']} | {r['n']} | {r['method']} | {'✓' if r['all_done'] else '✗'} | {_fmt(r['total_completion'])} "
                   f"| {_fmt(r['makespan'])} | {_fmt(r['collisions'], 0)} | {_fmt(r['min_separation'], 2)} "
                   f"| {_fmt(r['zone_wait'])} | {_fmt(r['stop_time'])} | {_fmt(r['reroutes'], 0)} "
                   f"| {_fmt(r['msgs_per_robot_s'])} | {_fmt(r['kbps_per_robot'], 2)} |")
    out += ["", "Methods: `swarmx` = CBBA + intent-aware routing + zone locks + ORCA/RSS; "
            "`stopwait` = traditional block reservation (stop-and-wait) with one-way lanes and greedy task claiming; "
            "`ghost` = robots ignore each other (unachievable lower bound); others are ablations.", ""]
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenarios", nargs="+", default=["crossing", "hot_aisles", "random", "blocked_aisle", "robot_failure"],
                    choices=sorted(SCENARIOS))
    ap.add_argument("--robots", nargs="+", type=int, default=[3, 5, 8])
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--methods", nargs="+", default=["swarmx", "stopwait"], choices=sorted(METHODS))
    ap.add_argument("--max-time", type=float, default=1500.0)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default="results")
    args = ap.parse_args(argv)

    jobs = [(sc, n, seed, m, args.max_time) for sc in args.scenarios for n in args.robots
            for seed in range(1, args.seeds + 1) for m in args.methods]
    t0 = time.time()
    print(f"running {len(jobs)} simulations on {args.jobs} workers ...", file=sys.stderr)
    runs = []
    with Pool(args.jobs) as pool:
        for i, r in enumerate(pool.imap_unordered(run_one, jobs), 1):
            runs.append(r)
            if i % 10 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}", file=sys.stderr)
    runs.sort(key=lambda r: (r["scenario"], r["n"], r["method"], r["seed"]))
    summary = summarise(runs)
    meta = {"date": time.strftime("%Y-%m-%d %H:%M"), "seeds": args.seeds, "runs": len(runs), "wall_s": time.time() - t0,
            "args": vars(args)}
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "benchmark.json"), "w") as f:
        json.dump({"meta": meta, "summary": summary, "runs": runs}, f, indent=1)
    md = to_markdown(summary, meta)
    with open(os.path.join(args.out, "benchmark.md"), "w") as f:
        f.write(md)
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
