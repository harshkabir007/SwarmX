# Typed per-robot telemetry (republished from the P2P state broadcast for
# RViz, rosbag and third-party tools).
std_msgs/Header header
string robot_id
geometry_msgs/Pose2D pose        # map frame
geometry_msgs/Vector3 velocity   # map frame, m/s
float32 battery                  # 0..1
string status                    # idle | to_pickup | picking | to_dropoff | dropping | to_charger | charging | parking | failed
string task
string[] bundle                  # CBBA task sequence
string waiting_zone
ZoneClaim[] zones
uint32 tasks_done
float32 distance_m
uint32 reroutes
