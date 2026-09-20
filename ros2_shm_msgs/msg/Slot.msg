# 单帧库位检测结果（替代 perception_idl/Slot）
std_msgs/Header header
shm_msgs/SlotData[] slots
# unix 时间戳，单位 ms
uint64 timestamp_ms