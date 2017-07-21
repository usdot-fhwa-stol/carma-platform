#
# TIM.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# TODO: Discuss the fields, types, and referenced msg files to use. 

# TIM.msg content.
# - msg_count
# - time_stamp *
# - packet_id * 
# - urlB *
# - DataFrames (TravelerDataFrameList.msg)
#	Part I Frame Header
#	Part II Applicable Regions of Use
#	Part III Content (including advisory, work-zone, speed limit info)
