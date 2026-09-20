# 单帧载具检测结果（替代 perception_idl/Pallet）
std_msgs/Header header
shm_msgs/PalletData[] pallets
# unix 时间戳，单位 ms
uint64 timestamp_ms