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
std_msgs/Header   header

# sender's static GUID (represents that vehicle throughout the current relationship)
string            sender_id

# a value from MobilityMessage which identifies this particular plan for future reference
string            plan_id

# the intention of this plan
cav_msgs/PlanType type

# a string of inputs from other vehicle
string            inputs

# plan expiration time in local clock
uint64            expiration
