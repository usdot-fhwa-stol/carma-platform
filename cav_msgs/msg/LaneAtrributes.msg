#
# LaneAttributes.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#

# directionalUse LaneDirection, -- directions of lane use
cav_msgs/LaneDirection directional_use

# sharedWith LaneSharing  -- co-users of the lane path

cav_msgs/LaneSharing shared_with

# laneType LaneTypeAttributes, -- specific lane type data
cav_msgs/LaneTypeAttributes laneType

# regional RegionalExtension {{REGION.Reg-LaneAttributes}} OPTIONAL
# TODO: Discuss how to implement

