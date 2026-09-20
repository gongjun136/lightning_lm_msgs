# 单帧可行驶区域检测（替代 perception_idl/DrivableArea）
std_msgs/Header header
# unix 时间戳，单位 ms
uint64 timestamp_ms
shm_msgs/FreeSpace[] freespace
shm_msgs/Curb[] curbs
shm_msgs/Occupancy occupancy