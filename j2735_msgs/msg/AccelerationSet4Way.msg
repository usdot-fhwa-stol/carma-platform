#
# AccelerationSet4Way.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# Set of acceleration values in 3 orthogonal directions of the vehicle and with yaw rotation rates expressed as a structure. 
# The positive longitudinal axis is to the front of the vehicle. 
# The positive lateral axis is to the right side of the vehicle (facing forward)
# Positive yaw is to the right (clockwise)
# The positive vertical "z" axis is downward with the zero point at the bottom of the vehicle's tires. 

#AccelerationSet4Way ::= SEQUENCE {
#   long Acceleration,          -- Along the Vehicle Longitudinal axis
#   lat  Acceleration,          -- Along the Vehicle Lateral axis
#   vert VerticalAcceleration,  -- Along the Vehicle Vertical axis
#   yaw  YawRate
#}

#Acceleration ::= INTEGER (-2000..2001) 
#   -- LSB units are 0.01 m/s^2
#   -- the value 2000 shall be used for values greater than 2000     
#   -- the value -2000 shall be used for values less than -2000  
#   -- a value of 2001 shall be used for Unavailable
#   -- Convert to m/s^2 with factor 0.01 when this field is used
int16 longitudinal
int16 lateral

int16 ACCELERATION_UNAVAILABLE = 2001
int16 ACCELERATION_MAX = 2000
int16 ACCELERATION_MIN = -2000

#VerticalAcceleration ::= INTEGER (-127..127) 
#   -- LSB units of 0.02 G steps over -2.52 to +2.54 G
#   -- The value +127 shall be used for ranges >= 2.54 G
#   -- The value -126 shall be used for ranges <= 2.52 G
#   -- The value -127 shall be used for unavailable
#   -- Convert to m/s^2 with factor 0.196 when this field is used
int8 vert

int8 ACCELERATION_VERTICAL_UNAVAILABLE = -127
int8 ACCELERATION_VERTICAL_MAX = 127
int8 ACCELERATION_VERTICAL_MIN = -126

#YawRate ::= INTEGER (-32767..32767)
#   -- LSB units of 0.01 degrees per second (signed)
#   -- Convert to degree with factor 0.01 when this field is used
int16 yaw_rate

int16 YAWRATE_UNAVAILABLE = 0
int16 YAWRATE_MAX = 32767
int16 YAWRATE_MIN = -32767
