#
# RouteSegment.msg
# 
# A route segment. If on the route/current_segment topic it is the active segment. 
#
# @author Michael McConnell
# @version 0.1
#

# The downtrack waypoint of this segment [upcoming]
cav_msgs/RouteWaypoint waypoint

# The uptrack waypoint of this segment [previous]
cav_msgs/RouteWaypoint prev_waypoint

# The length of this segment
float64 length

# The pose a FRD frame located in the ECEF frame. The FRD frame is located on the uptrack waypoint of this segment
# X-Axis: Along segment
# Y-Axis: Right of segment
# Z-Axis: Into ground (not necessarily toward earth center if there is a change in elevation)
# A pose is used instead of a transform as it will not be added to the tfTree and will have no frame id.
geometry_msgs/Pose FRD_pose