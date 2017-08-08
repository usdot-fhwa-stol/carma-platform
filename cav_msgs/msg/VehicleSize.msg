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
#Convert cm to meters. 
float32 vehicle_width
   
#VehicleLength ::= INTEGER (0.. 4095) -- LSB units of 1 cm with a range of >40 meters
#Convert cm to meters. 
float32 vehicle_length 