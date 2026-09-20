uint8 STATUS_IDLE=0
uint8 STATUS_INITIALIZING=1
uint8 STATUS_GOOD=2
uint8 STATUS_FOLLOWING_DR=3
uint8 STATUS_FAIL=4

# Message creation time. header.frame_id is unused.
std_msgs/Header header

# Current localization status.
uint8 status
