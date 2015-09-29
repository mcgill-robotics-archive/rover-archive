#Longitude in degrees
float32 gpsLongitude
#Latitude in degrees
float32 gpsLatitude
#Height above sea level in m
float32 gpsAltitude

#The pitch angle in radians from the euler angles
float32 pitch
#The roll angle in radians from the euler angles
float32 roll
#The yaw angle in radians from the euler angles
float32 yaw

#3d velocity in device coordinate and in m/s
geometry_msgs/Vector3 velocity

#GPS True heading accuracy value. (1e-5 unit)
int32 heading