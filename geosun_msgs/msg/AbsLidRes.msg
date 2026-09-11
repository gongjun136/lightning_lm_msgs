#  domain publisher /PosRes message, cycle 5ms
std_msgs/Header header
float64[3] f8xyz        #XYZ in ECEF: meter, precision: D-3
float64[9] f8deltaxyz   #XYZ协方差
float64[9] marb2e       #b系到e系的方向旋转矩阵
float64[9] f8deltapry   #方向旋转矩阵的协方差
