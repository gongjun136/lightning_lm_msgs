# 激光雷达数据定义（替代 sensor_idl/LidarFrame）
std_msgs/Header header
uint64 observe_timestamp
shm_msgs/SensorID sensor_id
uint32 point_num
shm_msgs/PointCloudData[] point_cloud