#
# PluginList.msg
# 
# Provided by the Guidance node. Returns a list of plugins depending on which
# topic is providing this message
#
# @author Kyle Rush
# @version 0.1
#

# A list of available plugins
std_msgs/Header header
cav_msgs/Plugin[] plugins
