# Trajectory.msg

# This message uses one start location in ECEF frame and a list of offset
# points relative to X, Y and Z of start point to represent a trajectory.
# The time interval between each pair of points should be 0.1 seconds.

# start location in ECEF frame
cav_msgs/LocationECEF          location

# a list of offsets describing current trajectory
cav_msgs/LocationOffsetECEF[]  offsets
