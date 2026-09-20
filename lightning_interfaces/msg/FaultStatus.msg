# Fault severity.
int8 LEVEL_NO_FAULT=-1
int8 LEVEL_P0=0
int8 LEVEL_P1=1
int8 LEVEL_P2=2

# Message creation time. header.frame_id is unused.
std_msgs/Header header

# Current fault severity.
int8 level

# Node-specific fault type. Zero means no fault.
int32 fault_type

# Human-readable description. Empty when there is no fault.
string description
