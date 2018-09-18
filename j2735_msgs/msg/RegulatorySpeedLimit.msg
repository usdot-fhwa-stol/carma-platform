#
# RegulatorySpeedLimit.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
# Used to convey a regulatory speed about a lane, lanes, or roadway segment.

# RegulatorySpeedLimit ::= SEQUENCE {
# type SpeedLimitType,
# -- The type of regulatory speed which follows
j2735_msgs/SpeedLimitType type

# speed Velocity
# Velocity ::= INTEGER (0..8191) -- Units of 0.02 m/s
#-- The value 8191 indicates that velocity is unavailable
float64 speed