# imu publish /ImuRealPitch_V_topic message, cycle 20ms
std_msgs/Header header       # ROS standard message header

float64   x         # 动臂俯仰角速度
float64   y         # 铲斗俯仰角速度 
float64   z         # 车身俯仰角速度