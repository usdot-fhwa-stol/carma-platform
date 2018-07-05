#
# Node-LLmD-64b.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# A 64-bit node type with lat-long values expressed in standard SAE one tenth of a micro degree.

# Node-LLmD-64b ::= SEQUENCE {
# lon Longitude,
# lat Latitude
# }

#Longitude ::= INTEGER (-1799999999..1800000001)
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 180 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
float64 longitude

float64 LONGITUDE_UNAVAILABLE = 180.0000001
float64 LONGITUDE_MAX = 180.0
float64 LONGITUDE_MIN = -179.9999999

#Latitude ::= INTEGER (-900000000..900000001)
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 90 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
float64 latitude

float64 LATITUDE_UNAVAILABLE = 90.0000001
float64 LATITUDE_MAX = 90.0
float64 LATITUDE_MIN = -90.0




