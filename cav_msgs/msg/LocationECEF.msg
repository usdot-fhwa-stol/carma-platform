# LocationECEF.msg

# This is used to describe a global location in
# "earth-centered, earth-fixed" (ECEF) frame in centimeters.
# Location unavailable can be represented by (0, 0, 0)

# X coordinate in ECEF frame
int32   ecef_x

# Y coordinate in ECEF frame
int32   ecef_y

# Z coordinate in ECEF frame
int32   ecef_z

# GPS timestamp in milliseconds
uint64  timestamp
