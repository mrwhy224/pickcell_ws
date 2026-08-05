uint8 UNKNOWN=0
uint8 UNCONFIGURED=1
uint8 INACTIVE=2
uint8 ACTIVE=3
uint8 DEGRADED=4
uint8 ERROR=5

std_msgs/Header header
string module_name
uint8 state
string implementation
string version
diagnostic_msgs/KeyValue[] details
