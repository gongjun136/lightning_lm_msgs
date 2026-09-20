# 静态目标（loader_perception_msgs/msg/StaticObject.msg）
# 字段对齐 perception_sdk::StaticObject，子类型用 ROS2 官方 geometry_msgs，
# 不依赖 rt_auto_idl / base_structure_idl。
#
# type 枚举值（uint8，与 perception_sdk::StaticObjectType 对齐）：
#   0=UNKNOWN  1=PALLET  2=SLOT  3=CONE

uint32 id
uint8 type
geometry_msgs/Point position
float32 confidence
float32 yaw
geometry_msgs/Vector3 size
