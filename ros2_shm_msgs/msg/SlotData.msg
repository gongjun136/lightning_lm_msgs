# 库位定义（替代 perception_idl/SlotData）
# SlotType: 0=PALLET 10=UNKNOWN 11=MARKED_FLOOR 12=STRACKED_PALLET 13=STRACKED_CONTAINER 14=CROSS_BEAM
# OccupiedStatus: 0=UNKNOWN 1=EMPTY 2=OCCUPIED 3=ABNORMAL_OCCUPIED 4=DISABLED
uint32 id
uint8 slot_type
geometry_msgs/Point position
bool has_height_limit
shm_msgs/EulerAngle orientation
geometry_msgs/Vector3 size
uint8 occupied_status
float32 occupied_status_conf
float32 confidence