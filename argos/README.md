# SwarmX on ARGoS3: large-scale swarm experiments

A C++ port of the SwarmX coordination logic for ARGoS foot-bots. It measures how the decentralized
approach scales to fleets of tens of robots, which is too many for Gazebo on one PC.

## What the port contains

| Part | Implementation in `controllers/swarmx_footbot/` |
|---|---|
| Communication | ARGoS range-and-bearing radio only: 2.5 m range, 2% packet loss, 32-byte payload. Neighbour *positions* come from the range/bearing measurement itself, so no shared localization is needed. |
| Task allocation | Distributed auction (CBBA with bundle size 1). A robot claims the cheapest open task that no heard neighbour out-bids it on. The task board (WMS) only records scans at the stations. |
| Planning | 8-connected A* with busy-aisle penalties and soft keep-right lanes in the cross aisles |
| Choke points | Aisle locks: pass-through convoys, exclusive otherwise, FIFO by expected entry time, settle window. Aisles a robot doesn't hold are walls. |
| Local motion | RVO2/ORCA (same linear programs as `swarmx_core/orca.py`), RSS safe-following guard, keep-right |
| Recovery | Idle robots park off the traffic lanes; a jam breaker re-plans around packed neighbours |
| Baseline (`mode="stopwait"`) | Stop if a robot is ahead; hard one-way lanes; aisle requested only once stopped at the mouth, exclusive; mutual-block back-off |

`loop_functions/` provides the task source, plus ground-truth metrics that robots never see: contacts
(centre distance below 2 × 8.5 cm), closest pass, makespan and total completion time. It writes
`results/<experiment>.json` and a `.csv` time series.

Arenas come from the same `swarmx_core.warehouse` model as the Python simulator and Gazebo. They are
scaled to 0.35 m per cell (single-lane aisles leave about 9 cm of clearance per side), and the floor
grows with the fleet (`--aisles`). Floors with more than 8 aisles get stations on both sides.

## Build and run

**Docker** (no ARGoS install needed):

```bash
docker build -f docker/Dockerfile.argos -t swarmx-argos docker
P=$PWD; docker run --rm --user $(id -u):$(id -g) -v $P:$P -w $P/argos swarmx-argos \
  bash -c "mkdir -p build && cd build && cmake .. && make"
docker run --rm --user $(id -u):$(id -g) -v $P:$P -w $P/argos swarmx-argos \
  python3 scripts/run_suite.py --sizes 10:8:30 30:16:90 50:24:150 --seeds 3
```

**Native** (after `sudo bash scripts/install_deps.sh`):

```bash
bash scripts/build_argos.sh        # ARGoS3 into ~/.local/argos3, then the SwarmX plugins
cd argos
python3 scripts/generate_experiment.py --robots 30 --aisles 16 --tasks 90 --visual
argos3 -c experiments/swarmx_30r_16a_swarmx_s1.argos      # Qt/OpenGL view
```

Use `--mode stopwait` for the baseline and `-z` for headless runs. The generated `.argos` files contain
absolute paths, so run them from the same checkout.
