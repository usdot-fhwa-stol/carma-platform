#!/usr/bin/env python
# license removed for brevity
#
# ROS Node converts CARMA ros topics into data which can be visualized with rviz
#
import rospy
from std_msgs.msg import String
from cav_msgs.msg import ExternalObjectList
from cav_msgs.msg import Route
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


from cav_msgs.msg import MobilityRequest
from cav_msgs.msg import MobilityPath

# Function to return current time
#current_time_millis = lambda: long(round(time.time() * 1000))

MPS_PER_MPH = 0.44704

# Node Class
class TopicVizNode(object):
  def __init__(self):
    
    # Constants
    self.CM_PER_M = 100.0
    self.MS_PER_S = 1000.0
    self.MOBILITY_TIMESTEP = 0.1

    # Init nodes
    rospy.init_node("topic_viz_node", anonymous=True)
    
    # Publishers

    # Subscribers
    # External Objects
    self.objects_viz_pub = rospy.Publisher("/external_obj_viz", MarkerArray, queue_size=10)
    self.objects_sub = rospy.Subscriber("/saxton_cav/sensor_fusion/filtered/tracked_objects", ExternalObjectList, self.external_objects_cb)
    # Roadway Objects. 
    # TODO this might require a java node
    # Route
    self.route_viz_pub = rospy.Publisher("/route_viz", PoseArray, queue_size=10, latch=True)
    self.route_sub = rospy.Subscriber("/saxton_cav/route/route", Route, self.route_cb)
    # Mobility Path
    self.request_viz_pub = rospy.Publisher("/mobility_request_viz", Marker, queue_size=10)
    self.path_sub = rospy.Subscriber("/saxton_cav/guidance/outgoing_mobility_path", MobilityPath, self.path_message_cb)
    # Mobility Request
    self.path_viz_pub = rospy.Publisher("/mobility_path_viz", Marker, queue_size=10)
    self.request_sub = rospy.Subscriber("/saxton_cav/guidance/outgoing_mobility_request", MobilityRequest, self.request_message_cb)

  # Function to convert request messages 
  def request_message_cb(self, request):
    print("\n\nRequest\n\n")
    msg_stamp = request.header.timestamp
    startPoint = request.trajectory.location
    offsets = request.trajectory.offsets
    blue = [0.0, 0.0, 1.0, 1.0] #RGBA
    pointMarker = self.pointsMarkerFromOffsets(msg_stamp, startPoint, offsets, self.MOBILITY_TIMESTEP, blue)

    self.path_viz_pub.publish(pointMarker)

  # Function to convert path messages 
  def path_message_cb(self, path):
    print("\n\nPath\n\n")
    msg_stamp = path.header.timestamp
    startPoint = path.trajectory.location
    offsets = path.trajectory.offsets
    green = [0.0, 1.0, 0.0, 1.0] #RGBA
    pointMarker = self.pointsMarkerFromOffsets(msg_stamp, startPoint, offsets, self.MOBILITY_TIMESTEP, green)

    self.path_viz_pub.publish(pointMarker)
  
  # Helper function converts a list of points in the ecef frame to poses
  def pointsMarkerFromOffsets(self, msg_stamp, startPoint, offsets, timestep, color):

    pointsMarker = Marker()
    pointsMarker.header.stamp = rospy.Time.from_sec(msg_stamp / self.MS_PER_S)
    pointsMarker.header.frame_id = "earth"
    # TODO set id so that multiple paths can be shown at once
    pointsMarker.type = Marker.POINTS
    pointsMarker.action = Marker.ADD
    pointsMarker.scale.x = 1.0
    pointsMarker.scale.y = 1.0
    pointsMarker.scale.z = 1.0
    pointsMarker.color.r = color[0]
    pointsMarker.color.g = color[1]
    pointsMarker.color.b = color[2]
    pointsMarker.color.a = color[3]
    #pointsMarker.lifetime = 0 #TODO

    points = []

    x = startPoint.ecef_x / self.CM_PER_M
    y = startPoint.ecef_y / self.CM_PER_M
    z = startPoint.ecef_z / self.CM_PER_M

    firstPoint = Point()
    firstPoint.x = x
    firstPoint.y = y
    firstPoint.z = z

    points.append(firstPoint)

    for offset in offsets:
      point = Point()
      x = x + (offset.offset_x / self.CM_PER_M)
      y = y + (offset.offset_y / self.CM_PER_M)
      z = z + (offset.offset_z / self.CM_PER_M)
      point.x = x
      point.y = y
      point.z = z

      points.append(point)

    pointsMarker.points = points

    return pointsMarker

  # Function to convert external object messages 
  def external_objects_cb(self, obj):
    print("\n\nObjects\n\n")
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
    self.objects_viz_pub.publish(markersArrayMsg)

  # Function to convert route messages 
  def route_cb(self, route):
    print("\n\nRoute\n\n")
    poses = []
    posesArray = PoseArray()
    posesArray.header.frame_id = "earth"
    posesArray.header.stamp = rospy.Time.now()
    for seg in route.segments:
      poses.append(seg.FRD_pose)
    posesArray.poses = poses
    self.route_viz_pub.publish(posesArray)
    
  # TODO remove
  def poseFromTransform(self, transform):
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.y = transform.translation.z
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w

    return pose

if __name__ == "__main__":
  try:
    tvn = TopicVizNode()
    # prevent python from exiting until this node is stopped
    rospy.spin()
  except rospy.ROSInterruptException:
    pass


    # firstPoint = Marker()
    # firstPoint.header.stamp = msg_stamp
    # firstPoint.header.frame_id = "earth"
    # firstPoint.type = Marker.SPHERE
    # firstPoint.action = Marker.ADD
    # firstPoint.pose.position.x = startPoint.ecef_x
    # firstPoint.pose.position.y = startPoint.ecef_x
    # firstPoint.pose.position.z = startPoint.ecef_x
    # firstPoint.scale.x = 1.0
    # firstPoint.scale.y = 1.0
    # firstPoint.scale.z = 1.0
    # firstPoint.color.r = 0.0
    # firstPoint.color.g = 0.0
    # firstPoint.color.b = 1.0
    # firstPoint.color.a = 1.0
    # firstPoint.lifetime = rospy.rostime.Duration(0.1) # The sphere will last 0.1 seconds