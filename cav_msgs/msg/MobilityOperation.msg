# MobilityOperation.msg

# This message is used by plugins to send necessary parameters to other
# CAVs at any desired frequency in order to accomplish a collaborative plan

# standard header for all mobility messages
cav_msgs/MobilityHeader  header

# strategy string to indicate which strategy type this message is used for
string                   strategy

# random GUID that identifies this particular strategy for future reference
# Example: "b937d2f6-e618-4867-920b-c1f74f98ef1f"
string                   plan_id

# strategy parameters specified by each individual plugin/component
# the maximum length of this string is 100
string                   strategy_params
