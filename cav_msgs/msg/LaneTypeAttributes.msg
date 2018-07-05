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

cav_msgs/LaneAttributes-Vehicle vehicle
cav_msgs/LaneAttributes-Crosswalk crosswalk
cav_msgs/LaneAttributes-Bike bikeLane
cav_msgs/LaneAttributes-Sidewalk sidewalk
cav_msgs/LaneAttributes-Barrier median
cav_msgs/LaneAttributes-Striping striping
cav_msgs/LaneAttributes-TrackedVehicle trackedVehicle
cav_msgs/LaneAttributes-Parking parking