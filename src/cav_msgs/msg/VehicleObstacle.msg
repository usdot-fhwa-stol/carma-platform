#
# VehicleObstacle.msg
# 
# Describes the geometry and movement of a vehicle in the roadway.
#
# @author Michael McConnell
# @version 0.1
#

# The geometry of the vehicle
cav_msgs/ExternalObject.msg

# Enum describing the type of communication this vehicle is capable of
cav_msgs/CommunicationClass communication_class

# A string representing what the communication functionalities of the vehicle are
string communicationProtocal