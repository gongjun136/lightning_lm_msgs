# 障碍物检测结果（shm_msgs/msg/Obstacle.msg）
# 对齐 perception_sdk::ObstacleResult，frame_id=base_link。
# 子字段类型用 ROS2 官方 geometry_msgs，不依赖 rt_auto_idl / perception_idl。

std_msgs/Header header
uint64 timestamp_ms
shm_msgs/DynamicObject[] dynamic_objects
shm_msgs/StaticObject[] static_objects
