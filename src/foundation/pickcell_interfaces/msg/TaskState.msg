uint8 IDLE=0
uint8 DETECTING=1
uint8 PLANNING_GRASP=2
uint8 PLANNING_MOTION=3
uint8 EXECUTING=4
uint8 RECOVERING=5
uint8 SUCCEEDED=6
uint8 FAILED=7
uint8 CANCELED=8

std_msgs/Header header
unique_identifier_msgs/UUID task_id
uint8 state
string stage
uint32 retry_count
float32 progress
string detail
