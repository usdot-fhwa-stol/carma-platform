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
import tf2_ros

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
    self.colors = [
      [1,0,0,1], # Red
      [1,0.65,0,1], # Orange
      [0,0,1,1], # Blue
      [1,1,0,1], # Yellow
      [0,1,1,1], # Cyan
      [1,0,1,1]  # Magenta
    ]

    self.host_color = [0,1,0,1] # Green

    # Init nodes
    rospy.init_node("topic_viz_node", anonymous=True)

    # TF
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # Pub-Sub
    # External Objects
    self.objects_viz_pub = rospy.Publisher("/external_obj_viz", MarkerArray, queue_size=10)
    self.objects_sub = rospy.Subscriber("/carma/sensor_fusion/filtered/tracked_objects", ExternalObjectList, self.external_objects_cb)
    # Roadway Objects. 
    # TODO this might require a java node
    # Route
    self.route_viz_pub = rospy.Publisher("/route_viz", PoseArray, queue_size=10, latch=True)
    self.route_sub = rospy.Subscriber("/carma/route/route", Route, self.route_cb)
    # Mobility Path
    self.path_viz_outbound_pub = rospy.Publisher("/mobility_path_viz_outbound", MarkerArray, queue_size=10)
    self.path_viz_inbound_pub = rospy.Publisher("/mobility_path_viz_inbound", MarkerArray, queue_size=10)
    self.path_sub_outbound = rospy.Subscriber("/carma/guidance/outgoing_mobility_path", MobilityPath, self.path_message_outbound_cb)
    self.path_sub_inbound = rospy.Subscriber("/carma/message/incoming_mobility_path", MobilityPath, self.path_message_inbound_cb)
    # Mobility Request
    self.request_viz_outbound_pub = rospy.Publisher("/mobility_request_viz_outbound", MarkerArray, queue_size=10)
    self.request_viz_inbound_pub = rospy.Publisher("/mobility_request_viz_inbound", MarkerArray, queue_size=10)
    self.request_sub_inbound = rospy.Subscriber("/carma/guidance/outgoing_mobility_request", MobilityRequest, self.request_message_outbound_cb)
    self.request_sub_outbound = rospy.Subscriber("/carma/message/incoming_mobility_request", MobilityRequest, self.request_message_inbound_cb)
    # Host Id
    self.host_veh_id = rospy.get_param("/carma/vehicle_id")
    
    # Color map for path / request messages
    self.id_color_map = dict({})

    # Location of map frame in earth
    self.mapInEarth = None

  # Helper function for processing path or request messages
  def process_msg_with_traj(self, msg, id_offset, pub):
    msg_stamp = msg.header.timestamp
    startPoint = msg.trajectory.location
    offsets = msg.trajectory.offsets

    color = self.getColorForId(msg.header.sender_id)
    pointMarker = self.pointsMarkerFromOffsets(msg_stamp, startPoint, offsets, self.MOBILITY_TIMESTEP, color, id_offset)

    pub.publish(pointMarker)

  # Function to convert request messages 
  def request_message_outbound_cb(self, request):
    self.process_msg_with_traj(request, 0, self.request_viz_outbound_pub)

  # Function to convert request messages 
  def request_message_inbound_cb(self, request):
    self.process_msg_with_traj(request, 0, self.request_viz_inbound_pub)

  # Function to convert path messages 
  def path_message_outbound_cb(self, path):
    self.process_msg_with_traj(path, 1000, self.path_viz_outbound_pub)

# Function to convert path messages 
  def path_message_inbound_cb(self, path):
    self.process_msg_with_traj(path, 1000, self.path_viz_inbound_pub)
  
  #Helper function converts a list of points in the ecef frame to poses
  def pointsMarkerFromOffsets(self, msg_stamp, startPoint, offsets, timestep, color, id_offset):

    markerArray = MarkerArray()
    markers = []

    points = []

    x = startPoint.ecef_x / self.CM_PER_M
    y = startPoint.ecef_y / self.CM_PER_M
    z = startPoint.ecef_z / self.CM_PER_M

    count = 0

    firstPoint = self.getCubeMarker(x,y,z,color,0.5,msg_stamp,"earth", len(self.id_color_map) + id_offset + count)

    markers.append(firstPoint)
    points.append(firstPoint)

    for offset in offsets:
      x = x + (offset.offset_x / self.CM_PER_M)
      y = y + (offset.offset_y / self.CM_PER_M)
      z = z + (offset.offset_z / self.CM_PER_M)

      marker = self.getCubeMarker(x,y,z,color,0.5,msg_stamp,"earth", len(self.id_color_map) + id_offset + count )
      markers.append(marker)

      count = count + 1

    markerArray.markers = markers

    return markerArray


  def getCubeMarker(self,x,y,z,color,scale,stamp,frame_id, marker_id):
    pointsMarker = Marker()
    pointsMarker.header.stamp = rospy.Time.from_sec(stamp / self.MS_PER_S)
    pointsMarker.header.frame_id = frame_id
    # TODO set id so that multiple paths can be shown at once
    pointsMarker.type = Marker.CUBE
    pointsMarker.action = Marker.ADD
    pointsMarker.scale.x = scale
    pointsMarker.scale.y = scale
    pointsMarker.scale.z = scale
    pointsMarker.color.r = color[0]
    pointsMarker.color.g = color[1]
    pointsMarker.color.b = color[2]
    pointsMarker.color.a = color[3]
    #pointsMarker.frame_locked = True
    pointsMarker.id = marker_id
    pointsMarker.lifetime = rospy.Time.from_sec(3.0)

    pointsMarker.pose.position.x = x
    pointsMarker.pose.position.y = y
    pointsMarker.pose.position.z = z
    pointsMarker.pose.orientation.w = 1.0 #required

    return pointsMarker
    
  # Helper function assigns a color to messages from a specific id
  def getColorForId(self, id):
    if (id == self.host_veh_id):
      return self.host_color
    if (self.id_color_map.has_key(id)):
      return self.id_color_map.get(id)
    else:
      numIds = len(self.id_color_map)
      if numIds < len(self.colors):
        colorForId = self.colors[numIds]
      else:
        colorForId = [1,1,1,1] # White default
      self.id_color_map[id] = colorForId
      return colorForId
  
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


if __name__ == "__main__":
  try:
    tvn = TopicVizNode()
    # prevent python from exiting until this node is stopped
    rospy.spin()
  except rospy.ROSInterruptException:
    pass


  # Old code using line strips
  #   # Helper function converts a list of points in the ecef frame to poses
  # def pointsMarkerFromOffsets(self, msg_stamp, startPoint, offsets, timestep, color, id_offset):

  #   # Get transform to rviz fixed frame (map)
  #   if (self.mapInEarth is None):
  #     try:
  #       self.mapInEarth = self.tfBuffer.lookup_transform("map", "earth", rospy.Time(0))
  #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
  #       return
    


  #   pointsMarker = Marker()
  #   pointsMarker.header.stamp = rospy.Time.from_sec(msg_stamp / self.MS_PER_S)
  #   pointsMarker.header.frame_id = "map"
  #   # TODO set id so that multiple paths can be shown at once
  #   pointsMarker.type = Marker.LINE_STRIP
  #   pointsMarker.action = Marker.ADD
  #   pointsMarker.scale.x = 0.5
  #   pointsMarker.scale.y = 0.5
  #   pointsMarker.scale.z = 1.0
  #   pointsMarker.pose = self.poseFromTransform(self.mapInEarth.transform)
  #   pointsMarker.color.r = color[0]
  #   pointsMarker.color.g = color[1]
  #   pointsMarker.color.b = color[2]
  #   pointsMarker.color.a = color[3]
  #   #pointsMarker.frame_locked = True
  #   pointsMarker.id = len(self.id_color_map) + id_offset
  #   #pointsMarker.lifetime = 0 #TODO

  #   points = []

  #   x = startPoint.ecef_x / self.CM_PER_M
  #   y = startPoint.ecef_y / self.CM_PER_M
  #   z = startPoint.ecef_z / self.CM_PER_M

  #   firstPoint = Point()
  #   firstPoint.x = x
  #   firstPoint.y = y
  #   firstPoint.z = z

  #   points.append(firstPoint)

  #   for offset in offsets:
  #     point = Point()
  #     x = x + (offset.offset_x / self.CM_PER_M)
  #     y = y + (offset.offset_y / self.CM_PER_M)
  #     z = z + (offset.offset_z / self.CM_PER_M)
  #     point.x = x
  #     point.y = y
  #     point.z = z

  #     points.append(point)

  #   pointsMarker.points = points

  #   return pointsMarker

  #
  # def poseFromTransform(self, transform):
  #   pose = Pose()
  #   pose.position.x = transform.translation.x
  #   pose.position.y = transform.translation.y
  #   pose.position.z = transform.translation.z
  #   pose.orientation.x = transform.rotation.x
  #   pose.orientation.y = transform.rotation.y
  #   pose.orientation.z = transform.rotation.z
  #   pose.orientation.w = transform.rotation.w

  #   return pose
