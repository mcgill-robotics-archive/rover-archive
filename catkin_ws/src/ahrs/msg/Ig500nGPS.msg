float64 latitude # degrees
float64 longitude # degrees
float64 altitude # meters

float64 vertical_accuracy # meters
float64 horizontal_accuracy # meters
float64 speed_accuracy # meters / seconds 
float64 heading_accuracy # degrees

geometry_msgs/Twist velocity # North, East, Down velocity in meters / seconds 
                             # (NED coordinate system)

uint8 FIX_TYPE_NO_FIX   = 0 # The GPS receiver does not have a fix at all
uint8 FIX_TYPE_FIX_TIME = 1 # The GPS receiver has a time only fix
uint8 FIX_TYPE_FIX_2D   = 2 # The GPS receiver has a 2D fix only
uint8 FIX_TYPE_FIX_3D   = 3 # Full 3D fix reception on the GPS receiver
uint8 fix_type 

bool validUTC # Valid UTC (Leap Seconds already known)

float64 heading # 2D heading in degrees
