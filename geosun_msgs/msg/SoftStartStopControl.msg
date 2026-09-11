# vcu subscribe /softStartStopControl_topic message, cycle 20ms
std_msgs/Header header       # ROS standard message header

int32   x        # 液压先导控制参数（域控液压先导请求）
int32   y        # 驻车控制参数（域控驻车制动请求）