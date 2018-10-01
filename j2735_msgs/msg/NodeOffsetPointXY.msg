#
# NodeOffsetPointXY.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_NodeOffsetPointXY data frame presents a structure to hold different sized data frames for a single node
# point in a lane. Nodes are described in terms of X and Y offsets in units of 1 centimeter (when zoom is 1:1).
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

j2735_msgs/NodeXY20b node_xy1
j2735_msgs/NodeXY22b node_xy2
j2735_msgs/NodeXY24b node_xy3
j2735_msgs/NodeXY26b node_xy4
j2735_msgs/NodeXY28b node_xy5
j2735_msgs/NodeXY32b node_xy6
j2735_msgs/NodeLLmD64b node_latlon
# regional #TODO: RegionalExtensions are not yet implemented in asn1c