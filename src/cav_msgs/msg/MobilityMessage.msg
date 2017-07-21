#
# MobilityMessage.msg
#
# @author Mae Fromm
# @version 0.1
#
# TODO: Discuss the fields, types, and referenced msg files t>> use. 


# MobilityIntro.msg : 
# -	version_number
# -	time_stamp
# -	request_id (unique request id)
# -	vehicle_id (requestor’s)
# -	vehicle_type
# -	roadway_segment
# -	lane_id
# -	latitude
# -	longtitude
# -	forward_speed
# -	public_key
# -	Mobilityoperation (MobilityoperationList.msg)
# >>	operation_id (e.g. Lane Change)
# >>	target_roadway_segment
# >>	target_lane_id
# >>	expiration_time
# 
# MobilityGreeting.msg:, (in response to another vehicle’s request)
# -	version_number
# -	time_stamp
# -	response_id (unique request id - random string S, encrypted w/ requestor’s public key)
# -	original_request_id (identify which unique request you’re responding to)
# -	vehicle_id (responder’s)
# -	vehicle_type
# -	roadway_segment
# -	lane_id
# -	latitude
# -	longtitude
# -	forward_speed
# -	public_key
# -	version_number
# -	time_stamp
# -	session_key (Encrypted with session key from the mobility_response.)
# 
# MobilityAcceptance.msg
# -	session_key (Encrypted with session key from the mobility_response.)
# 
# 
# MobilityProposal.msg
# -	version_number
# -	time_stamp
# -	session_key
# -	MobilityProposal.msg
# >>	proposal_id 
# >>	source_roadway
# >>	target_roadway
# >>	source_lane_id
# >>	target_lane_id
# >>	start_segment
# >>	start_segment_distance
# >>	end_segment
# >>	end_segment_distance
# >>	target_lateral_speed
# >>	session_key (encrypted with requestor’s public key)
# 
# MobilityDeal.msg (TBD)