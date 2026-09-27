# One SwarmX peer-to-peer message.
#
# All robots publish and subscribe on the single global topic /swarmx/p2p.
# With rmw_zenoh in peer mode (multicast scouting, no router) this is a true
# decentralized bus: there is no master, broker or central planner.
#
# The payload is the same compact JSON object used by the UDP multicast and
# Zenoh transports (see swarmx_core/protocol.py), so ROS robots, bare-metal
# edge robots and the simulator speak one protocol.
string src        # sender id, e.g. "robot3", "wms", "dashboard"
string type       # state | cbba | task | task_done | blocked | cmd
float64 stamp     # sender clock [s]
string payload    # JSON object
