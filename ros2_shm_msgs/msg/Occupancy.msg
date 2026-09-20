# Occupancy 定义（替代 perception_idl/Occupancy）
# occ_range/grid_size 原为 Vec2F(x,y)，这里用 geometry_msgs/Vector3，z 未用
geometry_msgs/Vector3 occ_range
geometry_msgs/Vector3 grid_size
# 占据概率图，行数
uint32 rows
# 占据概率图，列数
uint32 cols
# 占据概率，实际为二维数组
float32[] occ_prob