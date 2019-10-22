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

# Function to return current time
current_time_millis = lambda: long(round(time.time() * 1000))

MPS_PER_MPH = 0.44704

# Node Class
class TimeCorrectorNode(object):
    def __init__(self):
        # Init nodes
        rospy.init_node('time_corrector_node', anonymous=True)

        # Params
        self.dist_to_merge = rospy.get_param('~dist_to_merge')
        # Convert mph to mps
        self.speed_limit = rospy.get_param('~speed_limit') * MPS_PER_MPH
        
        # Publishers
        self.request_pub = rospy.Publisher('/carma/message/incoming_mobility_request', MobilityRequest, queue_size=10)
        self.path_pub = rospy.Publisher('/carma/message/incoming_mobility_path', MobilityPath, queue_size=10)

        # Subscribers
        self.request_sub = rospy.Subscriber("/rosbag/incoming_mobility_request", MobilityRequest, self.request_message_cb)
        self.path_sub = rospy.Subscriber("/rosbag/incoming_mobility_path", MobilityPath, self.path_message_cb)


    # Function to handle request messages from bag file
    def request_message_cb(self, request):

        # Estimate arrival time at lane change
        current_time = current_time_millis()
        futureTime = current_time + long(1000.0*(self.dist_to_merge / self.speed_limit))
        # Update message
        request.header.sender_id = "time_corrector_node"
        request.header.timestamp = futureTime
        request.expiration = futureTime + 500

        self.request_pub.publish(request)

    # Function to handle path messages from bag file
    def path_message_cb(self, path):

        path.header.sender_id = "time_corrector_node"
        path.header.timestamp = current_time_millis()

        self.path_pub.publish(path)

if __name__ == '__main__':
    try:
        tcn = TimeCorrectorNode()
        # prevent python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
