#  domain publisher /PosRes message, cycle 5ms
std_msgs/Header header
int32 i4week             #SYST week
float64 f8weeksecond     #SYST second
float64[3] f8enh         #ENH in Location: meter, precision: D-3
float64[3] f8xyz         #XYZ in ECEF: meter, precision: D-3
float64[3] f8blh         #latitude, longitude and height in WGS84: deg, precision: D-10, meter: D-3
float64[3] f8deltaenh    #the final positioning standard deviation in east, north and up directions: meter, precision: D-3
float64[3] f8venh        #plane speed: m/s
float64 f8vehiclespeed   #Vehicle Speed: m/s
float64[3] f8deltavenh   #//plane speed standard deviation: m/s
int32 i4type             #定位类型：EVENT(Image/INS): 0; GNSS Base: 1; GNSS Rove: 2; INS: 3; GNSS/INS: 10; WS/INS: 11;
                         #ZUPT: 12; TS/INS: 13; AbsLid/INS: 14; RelLid/INS: 15; AbsImage/INS: 16; RelImage/INS: 17 
float64[3] f8pry         #Euler angle: deg
float64[3] f8deltapry    #the standard deviation of Euler angle: deg
float64[3] f8angrate     #Gyro angle rate
float64[3] f8acce        #acceleration

