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
# TODO: Discuss the fields, types, and referenced msg files to use. 
# NOTE: Initial version only considers Part I. Part II will be defined for later use. 

# BSM.msg content:
#	BSMCoreData.msg (Part 1)
#	msg_count
#	id
#	sec_mark
#	lat
#	long
#	elev
#	Accuracy (PositionalAccuracy.msg)
#	Transmission (TransmissionState.msg)
#	speed
#	heading
#	angle
#	AccelSet (AccelerationSet4Way.msg)
#	Brake (BrakeSystemStatus.msg)
#	Size (VehicleSize.msg)
#	BSMPartIIContent.msg
