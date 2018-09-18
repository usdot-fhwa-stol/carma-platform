#
# NodeAttributeXY.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DE_NodeAttributeXY data element is an enumerated list of attributes which can pertain to the current node
# point.
#
# NodeAttributeXY ::= ENUMERATED {
# -- Various values which pertain only to the current node point
#
# -- General Items
# reserved,
# stopLine,		-- point where a mid-path stop line exists
# 			-- See also 'do not block' for segments
# -- Path finish details
# roundedCapStyleA, 	-- Used to control final path rounded end shape
# 		   	-- with edge of curve at final point in a circle
# roundedCapStyleB,	-- Used to control final path rounded end shape
# 			-- with edge of curve extending 50% of width past
# 			-- final point in a circle
#
# -- Topography Points (items with no concept of a distance along the path)
# mergePoint,		-- Japan merge with 1 or more lanes
# divergePoint,		-- Japan diverge with 1 or more lanes
# downstreamStopLine,	-- Japan style downstream intersection
# 			-- (a 2nd intersection) stop line
# downstreamStartNode, 	-- Japan style downstream intersection
# 			-- (a 2nd intersection) start node
#
# -- Pedestrian Support Attributes
# closedToTraffic,	-- where a pedestrian may NOT go
# 			-- to be used during construction events
# safeIsland,		-- a pedestrian safe stopping point
# 			-- also called a traffic island
# 			-- This usage described a point feature on a path,
# 			-- other entries can describe a path
# curbPresentAtStepOff, 	-- the sidewalk to street curb is NOT
# 			-- angled where it meets the edge of the
# 			-- roadway (user must step up/down)
# -- Lane geometry details (see standard for defined shapes)
# hydrantPresent,		-- Or other services access
# ...
# }


uint8  node_attribute_xy

# enumeration values for status:
uint8 RESERVED=0
uint8 STOPLINE=1
uint8 ROUNDEDCAPSTYLEA=2
uint8 ROUNDEDCAPSTYLEB=3
uint8 MERGEPOINT=4
uint8 DIVERGEPOINT=5
uint8 DOWNSTREAMSTOPLINE=6
uint8 DOWNSTREAMSTARTNODE=7
uint8 CLOSEDTOTRAFFIC=8
uint8 SAFEISLAND=9
uint8 CURBPRESENTATSTEPOFF=10
uint8 HYDRANTPRESENT=11
