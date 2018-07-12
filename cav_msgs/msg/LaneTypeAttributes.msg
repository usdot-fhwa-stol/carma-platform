#
# LaneTypeAttributes.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The Lane Type Attributes data frame is used to hold attribute information specific to a given lane type.
# LaneTypeAttributes ::= CHOICE {
# vehicle LaneAttributes-Vehicle, -- motor vehicle lanes
# crosswalk LaneAttributes-Crosswalk, -- pedestrian crosswalks
# bikeLane LaneAttributes-Bike, -- bike lanes
# sidewalk LaneAttributes-Sidewalk, -- pedestrian sidewalk paths
# median LaneAttributes-Barrier, -- medians & channelization
# striping LaneAttributes-Striping, -- roadway markings
# trackedVehicle LaneAttributes-TrackedVehicle, -- trains and trolleys
# parking LaneAttributes-Parking, -- parking and stopping lanes
# }

uint8 choice
uint8 VEHICLE=0
uint8 CROSSWALK=1
uint8 BIKE_LANE=2
uint8 SIDEWALK=3
uint8 MEDIAN=4
uint8 STRIPING=5
uint8 TRACKED_VEHICLE=6
uint8 PARKING=7

cav_msgs/LaneAttributesVehicle vehicle
cav_msgs/LaneAttributesCrosswalk crosswalk
cav_msgs/LaneAttributesBike bikeLane
cav_msgs/LaneAttributesSidewalk sidewalk
cav_msgs/LaneAttributesBarrier median
cav_msgs/LaneAttributesStriping striping
cav_msgs/LaneAttributesTrackedVehicle trackedVehicle
cav_msgs/LaneAttributesParking parking