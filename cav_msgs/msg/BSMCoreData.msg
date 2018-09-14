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

uint16 MSG_COUNT_MAX = 127

#TemporaryID ::= OCTET STRING (SIZE(4))
uint8[] id

#TemporaryID will change every 3000 seconds.
uint16 ID_TIME_MAX = 3000

# The presence vector is used to determine what optional items are set
# Existance can be quickly checked using bit operators
# Example msg.presence_vector & Msg.FLAG != 0
uint16 presence_vector

uint16 SEC_MARK_AVAILABLE = 1
uint16 LATITUDE_AVAILABLE = 2
uint16 LONGITUDE_AVAILABLE = 4
uint16 ELEVATION_AVAILABLE = 8
uint16 SPEED_AVAILABLE = 16
uint16 HEADING_AVAILABLE = 32
uint16 STEER_WHEEL_ANGLE_AVAILABLE = 64

#DSecond ::= INTEGER (0..65535)
#  -- Integer values from 0 to 59999 represent the milliseconds within a minute
#  -- A leap second is represented by the value range 60000 to 60999
#  -- The values from 61000 to 65534 are reserved
#  -- The value of 65535 shall represent an unavailable value in the range of the minute
#  -- Unit is milliseconds
uint16 sec_mark

uint16 SEC_MARK_MOD = 60000

#Latitude ::= INTEGER (-900000000..900000001)
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 90 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
float64 latitude

float64 LATITUDE_MAX = 90.0
float64 LATITUDE_MIN = -90.0

#Longitude ::= INTEGER (-1799999999..1800000001)
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 180 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
float64 longitude

float64 LONGITUDE_MAX = 180.0
float64 LONGITUDE_MIN = -179.9999999

#Elevation ::= INTEGER (-4096..61439)
#  -- In units of 10 cm steps above or below the reference ellipsoid
#  -- Providing a range of -409.5 to + 6143.9 meters
#  -- The value -4096 shall be used when Unknown is to be sent
#  -- Convert to meter with factor 0.1 when field is used
float32 elev

float32 ELEVATION_MAX = 6143.9
float32 ELEVATION_MIN = -409.5

# Used to model the accuracy of the positional determination with respect to each given axis.
cav_msgs/PositionalAccuracy accuracy

# Provide the current state of the vehicle transmission
j2735_msgs/TransmissionState transmission

#Speed ::= INTEGER (0..8191) -- Units of 0.02 m/s
#  -- The value 8191 indicates that speed is unavailable
#  -- Convert to m/s with factor 0.02 when field is used
float32 speed

float32 SPEED_MAX = 163.8
float32 SPEED_MIN = 0

#Heading ::= INTEGER (0..28800)
#  -- LSB of 0.0125 degrees
#  -- A range of 0 to 359.9875 degrees
#  -- Convert to degree with factor 0.0125 when field is used
float32 heading
float32 HEADING_MAX = 359.9875
float32 HEADING_MIN = 0

#SteeringWheelAngle ::= INTEGER (-126..127)
#  -- LSB units of 1.5 degrees, a range of -189 to +189 degrees
#  -- +001 = +1.5 deg
#  -- -126 = -189 deg and beyond
#  -- +126 = +189 deg and beyond
#  -- +127 to be used for unavailable
#  -- Convert to degree with factor 1.5 when this field is used
float32 angle
float32 STEER_WHEEL_ANGLE_MAX = 189.0
float32 STEER_WHEEL_ANGLE_MIN = -189.0

# Set of acceleration values in 3 orthogonal directions of the vehicle and with yaw rotation rates expressed as a structure.
cav_msgs/AccelerationSet4Way accelSet

# Conveys a variety of information about the current brake and system control activity of the vehicle.
j2735_msgs/BrakeSystemStatus brakes

# The vehicle length and width
cav_msgs/VehicleSize size
