# A decentralized choke-point lock entry (claim or hold) announced by a robot.
string zone       # e.g. "A12S"
string mode       # SN | NS (pass-through), SS | NN (enter-and-return), XX (exclusive)
string state      # claim | hold
float64 key       # precedence: expected entry time (lower goes first)
int32 lo          # zone-local cell interval the robot will occupy
int32 hi
string phase      # in | svc | out
int32 cur         # current zone-local cell (-1 = outside)
float32 eta       # seconds until the robot expects to have left the zone
