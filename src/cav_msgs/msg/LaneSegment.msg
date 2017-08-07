#
# LaneSegment.msg
# 
# Describes a small segment of a lane center line in 2d space.
#
# @author Michael McConnell
# @version 0.1
#

# The uptrack point of the lane segment
geometry_msgs/Point32.msg uptrackPoint

# The downtrack point of the lane segment
geometry_msgs/Point32.msg downtrackPoint

# the width of the lane
float32 width

# The LaneEdgeType of the right lane edge
cav_msgs/LaneEdgeType.msg rightSideType

# The LaneEdgeType of the left lane edge
cav_msgs/LaneEdgeType.msg leftSideType
