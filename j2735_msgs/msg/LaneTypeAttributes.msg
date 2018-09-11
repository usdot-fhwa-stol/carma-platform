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

j2735_msgs/LaneAttributesVehicle vehicle
j2735_msgs/LaneAttributesCrosswalk crosswalk
j2735_msgs/LaneAttributesBike bikeLane
j2735_msgs/LaneAttributesSidewalk sidewalk
j2735_msgs/LaneAttributesBarrier median
j2735_msgs/LaneAttributesStriping striping
j2735_msgs/LaneAttributesTrackedVehicle trackedVehicle
j2735_msgs/LaneAttributesParking parking