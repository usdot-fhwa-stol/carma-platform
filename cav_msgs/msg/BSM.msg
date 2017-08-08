#
# BSM.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# This is the parsed version of BSMRaw that will be passed around between Guidance, Negotiator, Message and other nodes. 
# 
# NOTE: Initial version only considers Part I. Part II will be defined for later use. 

std_msgs/Header header

cav_msgs/BSMCoreData core_data

# -- Part II Content is for future use. 



