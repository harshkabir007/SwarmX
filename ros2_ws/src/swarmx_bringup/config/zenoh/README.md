# Decentralized ROS 2 networking with rmw_zenoh

SwarmX robots share exactly one topic: `/swarmx/p2p` (best-effort, volatile).
Everything else (TF, scans, Nav2) stays inside each robot's namespace.

## Recommended: one Zenoh router per robot

```
robot A:  [nodes] -> router A  <== multicast discovery / gossip ==>  router B <- [nodes]  :robot B
```

* `swarmx_zenoh.env` selects rmw_zenoh. By default every process attaches to the
  router on its own host.
* On a real robot, start its router with `swarmx_zenoh_router_multirobot.env`, so that
  routers on different robots find each other by UDP multicast and connect router to router.
* No machine is special: there is no master, broker or discovery server. A robot that
  drops out of Wi-Fi keeps its local graph working, and its peers time out its claims and
  re-auction its tasks.

`sim_fleet.launch.py` starts one router for the simulation host automatically
(`zenoh_router:=true`).

## Pure peer mode (no routers)

`swarmx_zenoh_peer.env` puts every process in peer mode with multicast scouting.
This works and was verified with a talker/listener pair. In a full simulation with
about 40 processes on one host, though, some processes missed discovery and were
left isolated. That is why the per-robot router topology is the default.

## Troubleshooting

* `ros2 topic list` shows nothing: check `RMW_IMPLEMENTATION` in *every* shell and
  make sure a router is running on that host (`ros2 run rmw_zenoh_cpp rmw_zenohd`).
* Robots on different subnets: set `connect/endpoints` on the routers (see the env file).
