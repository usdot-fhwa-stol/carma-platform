#!/usr/bin/env python
# license removed for brevity
#
# ROS Node converts a mobility request or path message from a bag file to have an updated time stamp
# This causes that message to appear as if it had just been received
#
import time
import rospy

from cav_msgs.msg import MobilityRequest
from cav_msgs.msg import MobilityPath

request_pub = rospy.Publisher('/saxton_cav/message/incoming_mobility_request', MobilityRequest, queue_size=10)
path_pub = rospy.Publisher('/saxton_cav/message/incoming_mobility_path', MobilityPath, queue_size=10)

# Function to return current time
current_time_millis = lambda: int(round(time.time() * 1000))

def request_message_cb(request):

  request.header.sender_id = "time_corrector_node"
  request.header.timestamp = current_time_millis()
  request.expiration = current_time_millis() + 500

  request_pub.publish(request)

def path_message_cb(path):

  path.header.sender_id = "time_corrector_node"
  path.header.timestamp = current_time_millis()

  path_pub.publish(path)

def time_corrector_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('time_corrector_node', anonymous=True)

    rospy.Subscriber("/rosbag/incoming_mobility_request", MobilityRequest, request_message_cb)
    rospy.Subscriber("/rosbag/incoming_mobility_path", MobilityPath, path_message_cb)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        time_corrector_node()
    except rospy.ROSInterruptException:
        pass
