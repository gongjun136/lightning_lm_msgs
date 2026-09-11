# imu publish /Imu_RawYaw_topic message, cycle 20ms
std_msgs/Header header       # ROS standard message header

float64   x         # 动臂偏航角
float64   y         # 铲斗偏航角 
float64   z         # 车身偏航角
