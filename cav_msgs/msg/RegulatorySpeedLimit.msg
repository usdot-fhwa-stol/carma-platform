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

# Deviates from J2735 standard:
# Velocity in this message are conveyed as m/s rather than 0.02 m/s  to avoid redundant unnecessary conversion by subscribers
# Velocity ::= DOUBLE (0..1638.2) -- Units of m/s
#-- The value 1638.2 indicates that velocity is unavailable
float64 speed