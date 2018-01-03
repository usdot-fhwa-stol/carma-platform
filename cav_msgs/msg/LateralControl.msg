# LateralControl.msg
# 
# This message provides control commands for a lateral control driver
#

# Angle in radians to turn the wheels. Positive is left, Negative is right
float64 axel_angle

# Max axel_angle accerleration
float64 max_axel_angle_rate

# Max accerleration allowed to be caused by a turn
float64 max_accel