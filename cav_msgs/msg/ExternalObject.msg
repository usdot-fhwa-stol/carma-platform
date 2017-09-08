#
# ExternalObject.msg
# 
# Describes a tracked object

#Header contains the frame rest of the fields will use
Header header

#Object id. Matching ids on a topic should refer to the same object within some time period
uint16 id

#Pose of the object within the frame specified in header
geometry_msgs/PoseWithCovariance pose

#Average velocity of the object within the frame specified in header
geometry_msgs/TwistWithCovariance velocity

#Instantaneous velocity of an object within the frame specified in header
geometry_msgs/TwistWithCovariance velocity_inst

#The size of the object aligned along the axis of the object described by the orientation in pose
#Dimensions are specified in meters
geometry_msgs/Vector3 size

#Confidence [0,1]
float64 confidence

#range rate in m/s
float64 range_rate

#azimuth rate in rad/s
float64 azimuth_rate

uint8 relative_lane

# used for relative lane
uint8 HOST_LANE = 1
uint8 RIGHT_LANE = 2
uint8 LEFT_LANE = 3

#describes a general object type as defined in this message
uint8 object_type

#used for object type
uint8 UNKNOWN = 0
uint8 SMALL_VEHICLE = 1
uint8 LARGE_VEHICLE = 2
uint8 MOTORCYCLE = 3
uint8 PEDESTRIAN = 4

