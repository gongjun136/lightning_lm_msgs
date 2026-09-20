# 当前档位消息（/CurrentGear topic）
# 来自 V2D_ST3 (0x0CDFF005) 的 CurrentGear 信号
std_msgs/Header header        # ROS标准消息头

int32 gear                    # 当前档位: 0=R档, 1=N档, 2=F1档, 3=F2档
string gear_text              # 档位描述文本 (R档/N档/F1档/F2档)
