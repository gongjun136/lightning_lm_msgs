# 动态目标（loader_perception_msgs/msg/DynamicObject.msg）
# 字段对齐 perception_sdk::DynamicObject，子类型用 ROS2 官方 geometry_msgs，
# 不依赖 rt_auto_idl / base_structure_idl。
#
# type 枚举值（uint8，与 perception_sdk::DynamicObjectType 对齐）：
#   0=UNKNOWN  1=VEHICLE  2=PEDESTRIAN

uint32 id
uint8 type
geometry_msgs/Point position
float32 confidence
float32 yaw
geometry_msgs/Vector3 size
geometry_msgs/Vector3 velocity
bool is_static
