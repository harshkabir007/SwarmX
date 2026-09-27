# SwarmX results

All numbers are reproducible with the commands shown. The raw tables are in `docs/results/`.

## 1. Success criteria

| Criterion | Evidence | Status |
|---|---|---|
| Zero inter-robot collisions | 0 contacts in **90** lightweight-sim runs, **9** ARGoS runs (10–50 robots) and **4** Gazebo runs, all measured against simulator ground truth | **Met** |
| ≥ 20% reduction in total task completion time vs stop-and-wait on overlapping paths | Crossing scenario, reduction in the sum of task completion times: 8.3% (3 robots), 18.8% (5 robots), 77.5% (8 robots). Reduction in makespan (time until all tasks are done): 11.1% / **21.4%** / **80.5%**. | **Met at 8 robots on both measures; met at 5 robots for makespan only; not at 3** (see §2) |
| Decentralized communication | Robots share one P2P topic; no central node exists. Verified with separate processes over UDP multicast, with router-per-host rmw_zenoh in Gazebo, and with a range-limited lossy radio in ARGoS. | **Met** |
| Conflict resolution at choke points | Ground-truth aisle checks: 0 entries without a lock and 0 incompatible co-occupancies in all runs | **Met** |
| Task re-allocation and re-routing | Robot-failure and blocked-aisle scenarios: SwarmX completes every task, with 7–15% lower completion time than the baseline | **Met** |

## 2. Lightweight simulator: SwarmX vs stop-and-wait (6 seeds, 270 runs)

`python3 -m swarmx_core.sim.benchmark --seeds 6 --methods swarmx stopwait ghost`

**The baseline is deliberately strong.** It uses the traditional industrial scheme: block/cell reservation (a robot enters a cell only when it holds it, otherwise it stops and waits), hard one-way lanes, orthogonal moves, aisles requested only once stopped at their mouth, wait-for-cycle deadlock breaking, and greedy nearest-task claiming. It is collision-free too.

Reduction in total task completion time (positive = SwarmX faster):

| scenario | 3 robots | 5 robots | 8 robots |
|---|---:|---:|---:|
| crossing (overlapping paths) | 8.3% | 18.8% | 77.5%\* |
| hot aisles | 13.1% | 13.0% | 12.4% |
| random picks | 13.6% | 14.6% | 9.4% |
| blocked aisle | 14.2% | 13.3% | 8.4% |
| robot failure | 15.3% | 9.3% | 7.3% |

\* The stop-and-wait fleet gridlocked in some runs. Unfinished tasks are charged the time limit.

**How much improvement is even possible?** The `ghost` method lets robots pass through each other. That is physically impossible, which makes it the lower bound on any coordinator's completion time. On the crossing scenario (total completion time, s):

| robots | ghost bound | stop-and-wait | SwarmX | share of the avoidable delay SwarmX removes |
|---:|---:|---:|---:|---:|
| 3 | 814 | 1015 | 931 | 42% |
| 5 | 1360 | 2143 | 1740 | 51% |
| 8 | 2226 | 14386 | 3237 | 92% |

With 3 robots, stop-and-wait loses only 20% to conflicts in total, so a 20% reduction would require eliminating *every* conflict. The criterion is only achievable where paths overlap enough, which is exactly where SwarmX's margin grows.

## 3. ARGoS large-scale runs (3 seeds per cell)

`python3 argos/scripts/run_suite.py --sizes 10:8:30 30:16:90 50:24:150 --seeds 3`

The floor grows with the fleet, with 3 m of aisle per 3 robots. The radio has a 2.5 m range and 2% loss.

| robots | SwarmX delivered | SwarmX makespan | stop-and-wait delivered | reduction (total completion) | SwarmX collisions |
|---:|---:|---:|---:|---:|---:|
| 10 | 30/30 | 243 s | 30/30 | 9.3% | 0 |
| 30 | **90/90** | 534 s | 85/90 (gridlock) | 54.2% | 0 |
| 50 | **150/150** | 452 s | 119/150 (gridlock) | 77.7% | 0 |

The stop-and-wait baseline recorded one ground-truth contact in the three 50-robot runs; SwarmX recorded none.

## 4. Gazebo Harmonic end-to-end (ROS 2 Jazzy, Nav2, rmw_zenoh)

`ROBOTS=.. TASKS=.. LOC=.. EXEC=.. SCEN=.. bash scripts/gazebo_e2e.sh`

Every robot runs its own Nav2 stack and SwarmX agent. The only shared traffic is `/swarmx/p2p`, over one Zenoh router per host. Safety numbers come from Gazebo's true poses.

| robots | scenario | localization | executor | delivered | sim time | contacts | closest pass (body Ø 0.56 m) |
|---:|---|---|---|---:|---:|---:|---:|
| 3 | random | ground truth | direct | 6/6 | 178 s | 0 | 0.69 m |
| 3 | random | **AMCL** | direct | 6/6 | 181 s | 0 | 0.69 m |
| 3 | random | ground truth | **Nav2** (RPP) | 6/6 | 280 s | 0 | 0.80 m |
| 5 | **crossing** | ground truth | direct | 10/10 | 137 s | 0 | 0.80 m |

AMCL in rack aisles needed all 360 beams and a low odometry-noise model. With Nav2's defaults (60 beams, alpha 0.2) it slid up to 4.5 m along the featureless aisles. Real deployments of such warehouses usually add fiducials or reflectors.
