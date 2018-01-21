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

# a random value that identifies this particular plan for future reference
uint8[]             plan_id

# a list of proposed maneuvers
cav_msgs/Maneuver[] maneuvers
