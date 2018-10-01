#
# TrafficSignalInfo.msg
#
# Message Format for UI's Traffic Signal Plugin
#
# @author Mae Fromm
# @version 0.1


# IntersectionID that is unique within that region
uint16 intersection_id

# Outcome of cav_msgs/MovementState signal_state
uint8 state

uint8 UNLIT=0
uint8 GREEN=1
uint8 YELLOW=2
uint8 RED=3
uint8 FLASHING_GREEN=4
uint8 FLASHING_YELLOW=5
uint8 FLASHING_RED=6

# remaining time in seconds of the current state
uint16 remaining_time

# lane id when passing thru the intersection
uint16 lane_id

# remaining distance in meters of the intersection
float32 remaining_distance