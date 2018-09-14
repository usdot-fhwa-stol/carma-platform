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
int32 vehicle_width

int32 VEHICLE_WIDTH_UNAVAILABLE = 0.0
int32 VEHICLE_WIDTH_MAX = 1023
int32 VEHICLE_WIDTH_MIN = 1

#VehicleLength ::= INTEGER (0.. 4095) -- LSB units of 1 cm with a range of >40 meters
#  -- Convert to meter with factor 0.01 when this field is used
int32 vehicle_length

int32 VEHICLE_LENGTH_UNAVAILABLE = 0.0
int32 VEHICLE_LENGTH_MAX = 4095
int32 VEHICLE_LENGTH_MIN = 1
