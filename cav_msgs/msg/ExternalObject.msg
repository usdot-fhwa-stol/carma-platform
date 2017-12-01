#
# ExternalObject.msg
# 
# Describes a tracked object

#Header contains the frame rest of the fields will use
Header header

#a presence vector, this message is used to describe objects coming from potentially
#different sources. The presence vector is used to determine what items are set
#by the producer
uint16 presence_vector

uint16 ID_PRESENCE_VECTOR = 1
uint16 POSE_PRESENCE_VECTOR = 2
uint16 VELOCITY_PRESENCE_VECTOR = 4
uint16 VELOCITY_INST_PRESENCE_VECTOR = 8
uint16 SIZE_PRESENCE_VECTOR = 16
uint16 CONFIDENCE_PRESENCE_VECTOR = 32
uint16 RANGE_RATE_PRESENCE_VECTOR = 64
uint16 AZIMUTH_RATE_PRESENCE_VECTOR = 128
uint16 RELATIVE_LANE_PRESENCE_VECTOR = 256
uint16 OBJECT_TYPE_PRESENCE_VECTOR = 512
uint16 BSM_ID_PRESENCE_VECTOR = 1024

#Object id. Matching ids on a topic should refer to the same object within some time period, expanded
uint32 id

#bsm id is of the form [0xff,0xff,0xff,0xff]
uint8[] bsm_id

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

# lane object is detected in relative to the lane the sensor is detecting from
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

