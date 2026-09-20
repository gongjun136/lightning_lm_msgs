# 图像数据定义（替代 sensor_idl/ImageFrame）
std_msgs/Header header
uint64 observe_timestamp
shm_msgs/SensorID sensor_id
uint32 width
uint32 height
uint8 encoding
uint8 is_bigendian
uint8 data_type
uint32 step
# CUDA IPC 句柄，64 字节
uint8[64] data_ptr
# 测试场景图像数据（当 data_ptr 为 0 时使用）
uint8[] image_data