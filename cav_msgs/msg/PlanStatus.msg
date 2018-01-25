#
# PlanStatus.msg
# 
# Provided by the NegotiationReceiver plugin to indicate the status of
# proposed plan by NewPlan.msg from Negotiation node
#
# @author Shuwei Qiang
# @version 0.1
#

# standard ROS message header
std_msgs/Header header

# a value from MobilityMessage which identifies this particular plan for future reference
string          plan_id

# the status of indicated plan
bool            accepted
