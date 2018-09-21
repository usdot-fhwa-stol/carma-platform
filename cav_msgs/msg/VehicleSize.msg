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

# The presence vector is used to determine what optional items are set
# Existance can be quickly checked using bit operators
# Example msg.presence_vector & Msg.FLAG != 0
uint16 presence_vector

uint16 VEHICLE_WIDTH_AVAILABLE = 1
uint16 VEHICLE_LENGTH_AVAILABLE = 2

#VehicleWidth ::= INTEGER (0..1023) -- LSB units are 1 cm with a range of >10 meters
#  -- Convert to meter with factor 0.01 when this field is used
float32 vehicle_width

float32 VEHICLE_WIDTH_MAX = 10.23
float32 VEHICLE_WIDTH_MIN = 0.01

#VehicleLength ::= INTEGER (0.. 4095) -- LSB units of 1 cm with a range of >40 meters
#  -- Convert to meter with factor 0.01 when this field is used
float32 vehicle_length

float32 VEHICLE_LENGTH_MAX = 40.95
float32 VEHICLE_LENGTH_MIN = 0.01
