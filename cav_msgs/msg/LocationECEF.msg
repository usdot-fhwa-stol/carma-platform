# LocationECEF.msg

# This is used to describe a global location in
# "earth-centered, earth-fixed" (ECEF) frame in centimeters.
# Location unavaliable can be represented by (0, 0, 0)

# X cooridinate in ECEF frame
# the range for this value is [-638363700, 638363700]
int32   ecef_x

# Y cooridinate in ECEF frame
# the range for this value is [-638363700, 638363700]
int32   ecef_y

# Z cooridinate in ECEF frame
# the range for this value is [-636225200, 636225200]
int32   ecef_z

# GPS timestamp in milliseconds
uint64  timestamp
