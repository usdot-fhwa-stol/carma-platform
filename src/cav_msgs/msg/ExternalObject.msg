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