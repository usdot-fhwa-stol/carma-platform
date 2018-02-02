#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from cav_msgs.msg import ExternalObjectList
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

pub = rospy.Publisher('/external_pose', PoseArray, queue_size=10)

def callback(obj):
    poses = []
    posesArray = PoseArray()
    for obj in obj.objects:
        posesArray.header.stamp = obj.header.stamp
        posesArray.header.frame_id = obj.header.frame_id
        poseStamped = Pose()
        poseStamped = obj.pose.pose
        poses.append(poseStamped)
    posesArray.poses = poses
    pub.publish(posesArray)
        #poses.append(poseStamped)
    

    
def echo_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('echo_node', anonymous=True)

    rospy.Subscriber("/saxton_cav/sensor_fusion/filtered/tracked_objects", ExternalObjectList, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        echo_node()
    except rospy.ROSInterruptException:
        pass
