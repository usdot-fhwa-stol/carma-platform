#
# RoadwayEnvironment.msg
# 
# Provides a full picture of the roadway environment including other vehicles and lane geometry
#
# @author Michael McConnell
# @version 0.1
#

# A list of other vehicle obstacles on the roadway
cav_msgs/VehicleObstacle[] otherVehicles

# A list of lane geometries sorted by lane id
cav_msgs/Lane[] lanes

# A description of the host vehicle geometry
cav_msgs/VehicleObstacle hostVehicle