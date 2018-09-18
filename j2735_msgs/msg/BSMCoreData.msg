#
# BSMCoreData.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# Contains critical core elements deemed to be needed with every BSM issued.  
# 
# NOTE: Initial version only considers Part I. Part II will be defined for later use. 

#MsgCount ::= INTEGER (0..127)
uint8 msg_count

uint8 MSG_COUNT_MAX = 127

#TemporaryID ::= OCTET STRING (SIZE(4))
uint8[] id

#TemporaryID will change every 3000 seconds.
uint16 ID_TIME_MAX = 3000

#DSecond ::= INTEGER (0..65535)
#  -- Integer values from 0 to 59999 represent the milliseconds within a minute
#  -- A leap second is represented by the value range 60000 to 60999
#  -- The values from 61000 to 65534 are reserved
#  -- The value of 65535 shall represent an unavailable value in the range of the minute
#  -- Unit is milliseconds
uint16 sec_mark

uint16 SEC_MARK_MOD = 60000
uint16 SEC_MARK_UNAVAILABLE = 65535

#Longitude ::= INTEGER (-1799999999..1800000001)
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 180 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
int32 longitude

int32 LONGITUDE_UNAVAILABLE = 1800000001
int32 LONGITUDE_MAX = 1800000000
int32 LONGITUDE_MIN = -1799999999

#Latitude ::= INTEGER (-900000000..900000001)
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 90 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
int32 latitude

int32 LATITUDE_UNAVAILABLE = 900000001
int32 LATITUDE_MAX = 900000000
int32 LATITUDE_MIN = -900000000

#Elevation ::= INTEGER (-4096..61439)
#  -- In units of 10 cm steps above or below the reference ellipsoid
#  -- Providing a range of -409.5 to + 6143.9 meters
#  -- The value -4096 shall be used when Unknown is to be sent
#  -- Convert to meter with factor 0.1 when field is used
int32 elev

int32 ELEVATION_UNAVAILABLE = -4096
int32 ELEVATION_MAX = 61439
int32 ELEVATION_MIN = -4095

# Used to model the accuracy of the positional determination with respect to each given axis.
j2735_msgs/PositionalAccuracy accuracy

# Provide the current state of the vehicle transmission
j2735_msgs/TransmissionState transmission

#Speed ::= INTEGER (0..8191) -- Units of 0.02 m/s
#  -- The value 8191 indicates that speed is unavailable
#  -- Convert to m/s with factor 0.02 when field is used
uint16 speed

uint16 SPEED_UNAVAILABLE = 8191
uint16 SPEED_MAX = 8190
uint16 SPEED_MIN = 0

#Heading ::= INTEGER (0..28800)
#  -- LSB of 0.0125 degrees
#  -- A range of 0 to 359.9875 degrees
#  -- Convert to degree with factor 0.0125 when field is used
uint16 heading
uint16 HEADING_UNAVAILABLE = 28800
uint16 HEADING_MAX = 28798
uint16 HEADING_MIN = 0

#SteeringWheelAngle ::= INTEGER (-126..127)
#  -- LSB units of 1.5 degrees, a range of -189 to +189 degrees
#  -- +001 = +1.5 deg
#  -- -126 = -189 deg and beyond
#  -- +126 = +189 deg and beyond
#  -- +127 to be used for unavailable
#  -- Convert to degree with factor 1.5 when this field is used
int8 angle
int8 STEER_WHEEL_ANGLE_UNAVAILABLE = 127
int8 STEER_WHEEL_ANGLE_MAX = 126
int8 STEER_WHEEL_ANGLE_MIN = -126

# Set of acceleration values in 3 orthogonal directions of the vehicle and with yaw rotation rates expressed as a structure.
j2735_msgs/AccelerationSet4Way accelSet

# Conveys a variety of information about the current brake and system control activity of the vehicle.
j2735_msgs/BrakeSystemStatus brakes

# The vehicle length and width
j2735_msgs/VehicleSize size
