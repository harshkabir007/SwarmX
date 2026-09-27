"""SwarmX - decentralized coordination for fleets of autonomous mobile robots.

Pure-Python core shared by the lightweight simulator, the ROS 2 nodes and the
edge (Raspberry Pi) runtime:

* :mod:`swarmx_core.agent`      per-robot coordination brain (FleetAgent)
* :mod:`swarmx_core.cbba`       Consensus-Based Bundle Algorithm (task allocation)
* :mod:`swarmx_core.orca`       ORCA / RVO2 reciprocal collision avoidance
* :mod:`swarmx_core.zones`      decentralized choke-point locks
* :mod:`swarmx_core.planner`    grid A* + distance fields
* :mod:`swarmx_core.protocol`   P2P wire protocol
* :mod:`swarmx_core.transport`  in-process / UDP multicast / Zenoh transports
* :mod:`swarmx_core.warehouse`  shared warehouse model (+ SDF / Nav2 map export)
"""
__version__ = "0.1.0"
