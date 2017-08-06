#
# Plugin Message
# 
# The details of a discovered plugin
#
# @author Kyle Rush
# @version 0.1
#

std_msgs/Header header

# The string name identifier for the plugin
string name
# The strin version identifier for the plugin
string versionId
# Whether or not the plugin is currently available to generate a trajectory
bool available

