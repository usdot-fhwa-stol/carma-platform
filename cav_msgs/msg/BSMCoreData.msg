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

#TemporaryID ::= OCTET STRING (SIZE(4))
uint8[] id 

#DSecond ::= INTEGER (0..65535) 
#  -- Unit is milliseconds
#  -- Convert to seconds with factor 0.001 when field is used
uint16 sec_mark 

#Latitude ::= INTEGER (-900000000..900000001)  
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 90 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
float64 latitude           

#Longitude ::= INTEGER (-1799999999..1800000001)  
#   -- LSB = 1/10 micro degree
#   -- Providing a range of plus-minus 180 degrees
#   -- Convert to degree with factor 0.0000001 when field is used
float64 longitude             

#Elevation ::= INTEGER (-4096..61439) 
#  -- In units of 10 cm steps above or below the reference ellipsoid
#  -- Providing a range of -409.5 to + 6143.9 meters
#  -- The value -4096 shall be used when Unknown is to be sent
#  -- Convert to meter with factor 0.1 when field is used
float32 elev            

# Used to model the accuracy of the positional determination with respect to each given axis.
cav_msgs/PositionalAccuracy accuracy 

# Provide the current state of the vehicle transmission
cav_msgs/TransmissionState transmission  

#Speed ::= INTEGER (0..8191) -- Units of 0.02 m/s
#  -- The value 8191 indicates that speed is unavailable
#  -- Convert to m/s with factor 0.02 when field is used
float32 speed         

#Heading ::= INTEGER (0..28800) 
#  -- LSB of 0.0125 degrees
#  -- A range of 0 to 359.9875 degrees
#  -- Convert to degree with factor 0.0125 when field is used
float32 heading        

#SteeringWheelAngle ::= INTEGER (-126..127) 
#  -- LSB units of 1.5 degrees, a range of -189 to +189 degrees
#  -- +001 = +1.5 deg
#  -- -126 = -189 deg and beyond  
#  -- +126 = +189 deg and beyond
#  -- +127 to be used for unavailable
#  -- Convert to degree with factor 1.5 when this field is used
float32 angle        

# Set of acceleration values in 3 orthogonal directions of the vehicle and with yaw rotation rates expressed as a structure. 
cav_msgs/AccelerationSet4Way accelSet      

# Conveys a variety of information about the current brake and system control activity of the vehicle.
cav_msgs/BrakeSystemStatus brakes       

# The vehicle length and width 
cav_msgs/VehicleSize size     
