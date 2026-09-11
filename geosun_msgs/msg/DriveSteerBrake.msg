# vcu subscribe /Drive_Steer_Brake_cmd_topic message, cycle 20ms
std_msgs/Header header       # ROS standard message header

float64   x        # 前进/后退方向（域控挡位切换）
float64   y        # 转向指令信号（域控右转左转目标电流）
float64   z        # 刹车信号（域控制动踏板开度）
