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
float64 longitude
float64 latitude

#VerticalAcceleration ::= INTEGER (-127..127) 
#   -- LSB units of 0.02 G steps over -2.52 to +2.54 G
#   -- The value +127 shall be used for ranges >= 2.54 G
#   -- The value -126 shall be used for ranges <= 2.52 G
#   -- The value -127 shall be used for unavailable
float64 vert

#YawRate ::= INTEGER (-32767..32767) 
#   -- LSB units of 0.01 degrees per second (signed)
float64 yaw
