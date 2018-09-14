#
# VehicleSize.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The vehicle length and width 

#VehicleSize ::= SEQUENCE {
#   width     VehicleWidth,
#   length    VehicleLength
#   }
#

#VehicleWidth ::= INTEGER (0..1023) -- LSB units are 1 cm with a range of >10 meters
#  -- Convert to meter with factor 0.01 when this field is used
uint16 vehicle_width

uint16 VEHICLE_WIDTH_UNAVAILABLE = 0
uint16 VEHICLE_WIDTH_MAX = 1023
uint16 VEHICLE_WIDTH_MIN = 1

#VehicleLength ::= INTEGER (0.. 4095) -- LSB units of 1 cm with a range of >40 meters
#  -- Convert to meter with factor 0.01 when this field is used
uint16 vehicle_length

uint16 VEHICLE_LENGTH_UNAVAILABLE = 0
uint16 VEHICLE_LENGTH_MAX = 4095
uint16 VEHICLE_LENGTH_MIN = 1
