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

# The presence vector is used to determine what optional items are set
# Existance can be quickly checked using bit operators
# Example msg.presence_vector & Msg.FLAG != 0
uint16 presence_vector

uint16 ACCELERATION_AVAILABLE = 1
uint16 ACCELERATION_VERTICAL_AVAILABLE = 2
uint16 YAWRATE_AVAILABLE = 4

#Acceleration ::= INTEGER (-2000..2001) 
#   -- LSB units are 0.01 m/s^2
#   -- the value 2000 shall be used for values greater than 2000     
#   -- the value -2000 shall be used for values less than -2000  
#   -- a value of 2001 shall be used for Unavailable
#   -- Convert to m/s^2 with factor 0.01 when this field is used
float32 longitudinal
float32 lateral

float32 ACCELERATION_MAX = 20.0
float32 ACCELERATION_MIN = -20.0

#VerticalAcceleration ::= INTEGER (-127..127) 
#   -- LSB units of 0.02 G steps over -2.52 to +2.54 G
#   -- The value +127 shall be used for ranges >= 2.54 G
#   -- The value -126 shall be used for ranges <= 2.52 G
#   -- The value -127 shall be used for unavailable
#   -- Convert to m/s^2 with factor 0.196 when this field is used
float32 vert

float32 ACCELERATION_VERTICAL_MAX = 24.892
float32 ACCELERATION_VERTICAL_MIN = -24.696

#YawRate ::= INTEGER (-32767..32767)
#   -- LSB units of 0.01 degrees per second (signed)
#   -- Convert to degree with factor 0.01 when this field is used
float32 yaw_rate

float32 YAWRATE_MAX = 327.67
float32 YAWRATE_MIN = -327.67
