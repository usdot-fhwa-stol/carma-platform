#
# NewPlan.msg
# 
# Provided by the Negotiator node. Using the information from
# Mobility message to inform NegotiationReceiver plugin
#
# @author Shuwei Qiang
# @version 0.1
#

# standard ROS message header
std_msgs/Header     header

# a value from MobilityMessage which identifies this particular plan for future reference
string              plan_id

# a list of proposed maneuvers which we need to add into our trajectory
cav_msgs/Maneuver[] maneuvers

# plan expiration time in local clock
uint64                expiration
