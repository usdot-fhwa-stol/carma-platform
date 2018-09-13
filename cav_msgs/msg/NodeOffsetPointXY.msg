#
# NodeOffsetPointXY.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_NodeOffsetPointXY data frame presents a structure to hold different sized data frames for a single node
# point in a lane. 
#
# NodeOffsetPointXY ::= CHOICE {
# -- Nodes with X,Y content
# node-XY1 Node-XY-20b, -- node is within 5.11m of last node
# node-XY2 Node-XY-22b, -- node is within 10.23m of last node
# node-XY3 Node-XY-24b, -- node is within 20.47m of last node
# node-XY4 Node-XY-26b, -- node is within 40.96m of last node
# node-XY5 Node-XY-28b, -- node is within 81.91m of last node
# node-XY6 Node-XY-32b, -- node is within 327.67m of last node
# node-LatLon Node-LLmD-64b, -- node is a full 32b Lat/Lon range
# regional RegionalExtension {{REGION.Reg-NodeOffsetPointXY}}
# 	-- node which follows is of a
# 	-- regional definition type
# }
#

uint8 choice
uint8 NODE_XY1=0
uint8 NODE_XY2=1
uint8 NODE_XY3=2
uint8 NODE_XY4=3
uint8 NODE_XY5=4
uint8 NODE_XY6=5
uint8 NODE_LATLON=6

# Only set if choice != NODE_LATLON
# Deviates from J2735 standard:
# Latitude in this message are conveyed as m rather than cm to avoid redundant unnecessary conversion by subscribers
# Offset ::= FLOAT (-512..511)
# -- a range of +- 5.11 meters
# A delta offset in X, Y direction from some known point. For non-vehicle centric coordinate frames of
# reference, offset is positive to the East (X) and to the North (Y) directions. The most negative value shall be used to
# indicate an unknown value.

float32 x
float32 y

# Only set if choice == NODE_LATLON
# Deviates from J2735 standard:
# Latitude in this message are conveyed as deg rather than 1/10th micro deg to avoid redundant unnecessary conversion by subscribers
#Longitude ::= INTEGER (-179.9999999..180.0000001)
#  -- Providing a range of plus-minus 180 degrees
float64 longitude

float64 LONGITUDE_UNAVAILABLE = 180.0000001
float64 LONGITUDE_MAX = 180.0
float64 LONGITUDE_MIN = -179.9999999

# Only set if choice == NODE_LATLON
# Deviates from J2735 standard:
# Latitude in this message are conveyed as deg rather than 1/10th micro deg to avoid redundant unnecessary conversion by subscribers
#Latitude ::= INTEGER (-90.0000000-90.0000001)
#  -- Providing a range of plus-minus 90 degrees
float64 latitude

float64 LATITUDE_UNAVAILABLE = 90.0000001
float64 LATITUDE_MAX = 90.0
float64 LATITUDE_MIN = -90.0

# regional #TODO: RegionalExtensions are not yet implemented in asn1c