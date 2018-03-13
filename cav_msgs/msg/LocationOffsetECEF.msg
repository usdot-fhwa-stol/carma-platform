# LocationOffsetECEF.msg

# Location offset is used to describe a trajectory based on
# the combination of start location and a list of offsets.
# Each offset inficates the offset from last offset/location in
# centimeters. The time interval between each pair of neighbor
# points should be 0.1 second. So the length of trajectory
# will depend on the vehicle current speed.

# offset in X cooridinate of ECEF frame
# the x, y, z range is in [-500, 501] and 501 means offset unavaliable
int16  offset_x

# offset in Y cooridinate of ECEF frame
int16  offset_y

# offset in Z cooridinate of ECEF frame
int16  offset_z
