uint8 UNKNOWN=0
uint8 SAFE=1
uint8 DEGRADED=2
uint8 STOP_REQUESTED=3
uint8 STOPPED=4
uint8 FAULT=5

std_msgs/Header header
uint8 state
bool motion_authorized
string[] active_interlocks
string reason
