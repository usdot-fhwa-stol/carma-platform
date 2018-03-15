# MobilityRequest.msg

# This is the message a CAV will use to send out its request to the other CAVs
# when a coordinated behavior is required. It will require responses from
# CAVs. If the request is boardcast, then no response means positive response;
# if the request is only sent to one CAV, then no response means negative response

# standard header for all mobility messages
cav_msgs/MobilityHeader        header

# strategy string to indicate which strategy type this message is used for
# Example: "Carma/Platooning"
string                         strategy

# type of the proposal being suggested to neighbors
cav_msgs/PlanType              plan_type

# urgency of the current plan from 0 to 1000
# 0 means no urgency specified and 1000 means the highest urgency
uint16                         urgency

# current location in ECEF frame
cav_msgs/LocationECEF          location

# strategy parameters specified by each individual plugin/component
# the maximum length of this string is 100
string                         strategy_params

# proposed trajectory
cav_msgs/Trajectory            trajectory

# time when this message is expired
uint64                         expiration
