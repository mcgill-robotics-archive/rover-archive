# Locomotor state

std_msgs/Header header

# Internal Command ID
uint32 cmd_id

# Current status
int8 status
int8 FAULT      = -1  # The rover enconters error during execution
int8 OFF        =  0  # The locomotor is off
int8 STATIONARY =  1  # The rover is idle; waiting for a command
int8 MOVING     =  2  # The rover is executing the command

# Fraction of complete
float32 fraction_complete_driving
float32 fraction_complete_steering

# Estimated completion time
float32 estimated_time_driving
float32 estimated_time_steering
