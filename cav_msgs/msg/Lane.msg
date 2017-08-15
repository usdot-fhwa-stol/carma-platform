#
# Lane.msg
# 
# Describes the geometry of a lane on the roadway in a local 2d projection
#
# @author Michael McConnell
# @version 0.1
#

# The lane index of this lane as defined by the route package lane index standard
uint8 laneIndex

# The segments of the lane
cav_msgs/LaneSegment[] laneSegments
