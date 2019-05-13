#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from cav_msgs.msg import ExternalObjectList
from cav_msgs.msg import Route
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Transform
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

pub = rospy.Publisher('/external_obj_markers', MarkerArray, queue_size=10)
route_pub = rospy.Publisher('/route_pose', PoseArray, queue_size=10)

def external_objects_cb(obj):
    markers = []
    markersArrayMsg = MarkerArray()
    for obj in obj.objects:
        marker = Marker()
        marker.header.stamp = obj.header.stamp
        marker.header.frame_id = obj.header.frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose = obj.pose.pose
        marker.scale = obj.size
        marker.frame_locked = True
        marker.lifetime = rospy.Duration.from_sec(1)
        # Color of blue
        marker.color.a = 1.0 # Alpha of 1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        markers.append(marker)
    markersArrayMsg.markers = markers
    pub.publish(markersArrayMsg)

def route_cb(route):
    poses = []
    posesArray = PoseArray()
    pose = route.segments[0].pose
    poses.append(pose)
    for seg in route.segments:
        posesArray.header.frame_id = seg.pose.header.frame_id
        posesArray.header.stamp = seg.pose.header.stamp
        poses.append(seg.pose)
    posesArray.poses = poses
    route_pub.publish(posesArray)
    
def poseFromTransform(transform):
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.y = transform.translation.z
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w

    return pose
    
def echo_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('echo_node', anonymous=True)

    rospy.Subscriber("/carma/sensor_fusion/filtered/tracked_objects", ExternalObjectList, external_objects_cb)
    #rospy.Subscriber("/carma/route/route", Route, route_cb)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        echo_node()
    except rospy.ROSInterruptException:
        pass
