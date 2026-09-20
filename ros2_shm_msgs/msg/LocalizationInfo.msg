# 定位消息体（替代 localization_idl/LocalizationInfo）
std_msgs/Header header
uint64 observe_timestamp
# LocalizationStatus
uint8 STATUS_UNAVAILABLE = 0
uint8 STATUS_AVAILABLE = 1
uint8 STATUS_INIT = 2
uint8 status
# LocalizationErrorCode
uint8 ERROR_NO_ERROR = 0
uint8 ERROR_SENSOR_UNAVAILABLE = 1
uint8 error_code
geometry_msgs/Pose pose
shm_msgs/EulerAngle attitude
geometry_msgs/Vector3 speed
geometry_msgs/Vector3 std_position
geometry_msgs/Vector3 std_attitude