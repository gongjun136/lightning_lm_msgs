#  domain publisher /PosPoint_topic message, cycle 20ms
std_msgs/Header header
int32      pos_status
int32      pos_type
float64[3] blh  # BLH 
float64[3] sigma_blh
float64    diff_age
int32      vph_status
int32      vph_type
float64[2] vph       
float64[2] sigma_vph
float64    azimuth 
float64    sigma_azi 

