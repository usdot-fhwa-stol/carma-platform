# LayerType.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# Data element used to uniquely identify the type of information to be found in a layer of geographic map fragment
# such as an intersection.
#

uint8 layer_type

# LayerType ::= ENUMERATED {
# none,
# mixedContent, -- two or more of the below types
# generalMapData,
# intersectionData,
# curveData,
# roadwaySectionData,
# parkingAreaData,
# sharedLaneData,
# ...
# }

# enumeration values for plan type:
uint8       NONE = 0
uint8       MIXED_CONTENT = 1
uint8       GENERAL_MAP_DATA = 2
uint8       INTERSECTION_DATA = 3
uint8       CURVE_DATA = 4
uint8       ROADWAY_SECTION_DATA = 5
uint8       PARKING_AREA_DATA = 6
uint8       SHARED_LANE_DATA = 7