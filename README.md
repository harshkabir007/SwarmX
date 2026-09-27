# SwarmX: decentralized fleet coordination for warehouse AMRs

Edge-AI fleet coordination for autonomous mobile robots (problem statement 26123, Bharat Electronics Limited).
Every robot runs its own copy of the coordination software. There is no central server, broker or
planner. Robots share position and intent peer-to-peer, auction tasks among themselves, negotiate
single-lane aisles, and avoid each other locally.

| Requirement | How SwarmX does it |
|---|---|
| Decentralized communication | One P2P topic `/swarmx/p2p` over **rmw_zenoh** (one router per robot, routers discover each other). The same JSON protocol also runs over UDP multicast or Zenoh without ROS. |
| Conflict resolution at choke points | **Decentralized aisle locks**: direction-aware convoys, LIFO stacking, disjoint-extent sharing, FIFO by expected entry time. Deadlock-free by construction and enforced at the motion level. |
| Collision avoidance | **ORCA (RVO2 port)** plus an RSS-style safe-following guard, keep-right rule, and Nav2 collision monitor as the last layer |
| Task allocation and re-routing | **CBBA** consensus auction (task locking, failed-robot timeout, re-bidding); **intent-aware routing**, where robots announce upcoming aisle traversals with time windows and others route around opposite traffic; obstacle reports with decay |
| Fleet dashboard | Lightweight web UI (vanilla JS, stdlib WebSocket/REST server) that is a *passive* P2P listener: live map, battery, states, tasks, events |
| Edge hardware | Pure-Python core, no numpy, no pip dependencies. `swarmx_core.edge` runs one robot on a Raspberry Pi without ROS. |

## Repository layout

```
ros2_ws/src/
  swarmx_core/         pure-Python core: agent, CBBA, ORCA, zone locks, planner, P2P protocol,
                       transports (in-process / UDP / Zenoh), simulator, benchmark, dashboard, edge runtime
  swarmx_interfaces/   ROS 2 messages (P2P envelope, RobotState, ZoneClaim)
  swarmx_fleet/        ROS 2 fleet agent node (per robot), lidar perception, task-source (WMS) peer
  swarmx_dashboard/    ROS 2 dashboard node (passive peer, spawns physical pallets in Gazebo)
  swarmx_description/  URDF/xacro diff-drive AMR with Gazebo Harmonic plugins
  swarmx_gazebo/       warehouse world generated from the shared warehouse model, robot spawning
  swarmx_navigation/   per-robot Nav2: AMCL / slam_toolbox, velocity smoother + collision monitor, optional full Nav2
  swarmx_bringup/      one-command fleet launch, real-robot launch, rmw_zenoh configs, RViz
argos/                 ARGoS3 large-scale experiments (C++ foot-bot port of the coordination logic)
docker/                Dockerfiles: ROS 2 Jazzy + Nav2 + slam_toolbox + rmw_zenoh, and ARGoS3
scripts/               install_deps.sh (sudo), build_ws.sh, build_argos.sh
```

The Gazebo world, the Nav2 map and the fleet's planning graph are all generated from one model
(`swarmx_core/warehouse.py`), so they cannot drift apart.

## Quick start without ROS (any machine with Python 3.8+)

```bash
cd ros2_ws/src/swarmx_core
python3 -m swarmx_core.sim.run --robots 6                  # live dashboard on http://localhost:8080
python3 -m swarmx_core.sim.run --scenario crossing --robots 8 --method stopwait   # watch the baseline
python3 -m swarmx_core.sim.benchmark --seeds 6             # SwarmX vs stop-and-wait -> results/benchmark.md
python3 -m pytest test                                     # 33 tests
```

From the dashboard you can pause or speed up the simulation, add tasks, drop obstacles into aisles, fail
and recover robots, or restart a scenario.

**A real P2P fleet over UDP multicast.** Run these in separate terminals, or on separate Pis on the same Wi-Fi:

```bash
python3 -m swarmx_core.edge --id robot1 --start 3 6
python3 -m swarmx_core.edge --id robot2 --start 4 6
python3 -m swarmx_core.edge --id robot3 --start 3 13
python3 -m swarmx_core.edge --wms --tasks 12                # task source peer
python3 -m swarmx_core.edge --dashboard --port 8080         # passive dashboard peer
```

## ROS 2 Jazzy + Gazebo Harmonic

Ubuntu 24.04:

```bash
sudo bash scripts/install_deps.sh      # ROS 2 Jazzy, Gazebo Harmonic (ros_gz), Nav2, slam_toolbox, rmw_zenoh, ARGoS deps
bash scripts/build_ws.sh
source ros2_ws/install/setup.bash
source ros2_ws/install/swarmx_bringup/share/swarmx_bringup/config/zenoh/swarmx_zenoh.env
ros2 launch swarmx_bringup sim_fleet.launch.py robots:=3              # Gazebo + RViz + dashboard on :8080
ros2 launch swarmx_bringup sim_fleet.launch.py robots:=5 scenario:=crossing localization:=amcl gui:=false
```

Useful launch arguments:

- `localization:=static|amcl|slam_toolbox`
- `executor:=direct|nav2`: in `direct`, SwarmX drives through Nav2's smoother and collision monitor; in `nav2`, Nav2 plans and tracks the waypoints SwarmX releases.
- `tasks`, `scenario`, `headless_rendering`

On a physical robot, see `ros2 launch swarmx_bringup robot.launch.py` and `swarmx_bringup/config/zenoh/README.md`.

**No ROS install? Use Docker:**

```bash
docker build -f docker/Dockerfile -t swarmx .
docker run --rm -it --network host swarmx
# inside the container:
ros2 launch swarmx_bringup sim_fleet.launch.py gui:=false headless_rendering:=true rviz:=false
```

## Results (lightweight simulator, 6 seeds per cell, 90 SwarmX runs)

Reduction in **total task completion time** versus a traditional stop-and-wait baseline. The baseline
is a competent one: block/cell reservation, one-way lanes, wait-for-cycle deadlock breaking and greedy
task claiming, all collision-free.

| scenario | 3 robots | 5 robots | 8 robots |
|---|---:|---:|---:|
| crossing (overlapping paths through shared aisles) | 8.3% | 18.8% | 77.5%\* |
| hot aisles | 13.1% | 13.0% | 12.4% |
| random picks | 13.6% | 14.6% | 9.4% |
| blocked aisle (pallets dropped mid-run) | 14.2% | 13.3% | 8.4% |
| robot failure (dies at 30 s, reboots at 150 s) | 15.3% | 9.4% | 9.0% |

\* The stop-and-wait fleet gridlocked in some 8-robot crossing runs. Unfinished tasks are charged the time limit.

- **Safety: 0 collisions, 0 aisle entries without a lock and 0 incompatible aisle co-occupancies**, all checked against simulator ground truth.
- The 20% target is reached on overlapping paths at 8 robots. At 5 robots it comes close (18.8%).
- At 3 robots, the interference-free "ghost" lower bound shows the baseline loses only about 18% to conflicts, so no coordinator could reach 20% there.

Reproduce with `python3 -m swarmx_core.sim.benchmark --seeds 6`. Ablations are available: `--methods swarmx stopwait ghost swarmx_greedy swarmx_no_intent`.

**Gazebo end-to-end (verified in the Docker image).** 3 robots, each with its own Nav2 safety chain and
SwarmX agent, coordinated only over `/swarmx/p2p` via rmw_zenoh. They delivered all 6 tasks with no
false obstacle reports.

## How it works (short)

- **Per robot** (`swarmx_core/agent.py`), at 10–20 Hz:
  - Receive peer broadcasts.
  - Run CBBA to decide what to do.
  - Plan an A* route with intent-aware aisle penalties.
  - Claim, hold and release aisle locks.
  - Compute an ORCA velocity with the RSS guard.
  - Broadcast its own state and intent.
- **Aisle locks** (`zones.py`) need no lock server. Every robot applies the same deterministic rule to the claims it hears.
  - Precedence is fixed at claim time, which makes it a total order.
  - Robots wait only outside aisles.
  - Aisles a robot does not hold are walls to its ORCA, so traffic can never push it into one.
- **Perception** (`swarmx_fleet/perception.py`):
  - Lidar returns in free map cells that no radio peer explains become obstacle reports, which are shared P2P and expire unless re-confirmed.
  - Robot-shaped returns without a radio link (for example a dead robot) are avoided with full responsibility.
  - Scans are projected from the lidar pose *at scan time*.

## Status and known limitations

- **Verified:** core algorithms and tests; the benchmark; the UDP multi-process fleet; the Gazebo fleet (`localization:=static`, `executor:=direct`, rmw_zenoh with a router per host); the dashboard (simulation and ROS).
- **Written but not yet exercised end-to-end in Gazebo:** AMCL and slam_toolbox localization modes, the `nav2` executor, and physical pallets spawned from the dashboard.
- **ARGoS** (`argos/`): builds and runs headless (10 foot-bots, 0 collisions). In the first 30-task run, 27 tasks were delivered, so work is still in progress.
- Pure router-less Zenoh peer mode (`swarmx_zenoh_peer.env`) works for small graphs. With about 40 processes on one host, some peers missed discovery, which is why the router-per-robot topology is the default.
