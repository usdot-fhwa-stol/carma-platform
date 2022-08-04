#!/usr/bin/env python3

# Copyright (C) 2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

# Worker class for core business logic
class PoseOffsetWorker:
    def __init__(self, pub, x_offset, y_offset, z_offset):
        self.pub = pub
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
    
    # Callback to apply offset based on parameters
    def pose_cb(self, msg):
      msg.pose.position.x += self.x_offset
      msg.pose.position.y += self.y_offset
      msg.pose.position.z += self.z_offset

      self.pub.publish(msg)

    # Method to update parameters at runtime based on topic input
    def param_cb(self, msg):
      self.x_offset = msg.x
      self.y_offset = msg.y
      self.z_offset = msg.z
      

  

def run():

  rospy.init_node('pose_offset_node')

  # Load params
  x_offset = rospy.get_param("/pose_x_offset", 1.0)
  y_offset = rospy.get_param("/pose_y_offset", 1.0)
  z_offset = rospy.get_param('/pose_z_offset', 0.0)

  # Setup publishers
  pub = rospy.Publisher('selected_pose', PoseWithCovarianceStamped, queue_size=10)
  
  # Create worker
  wkr = PoseOffsetWorker(pub, x_offset, y_offset, z_offset)

  # Bind subscriptions to worker
  sub = rospy.Subscriber('old_selected_pose', PoseWithCovarianceStamped, wkr.pose_cb, queue_size=10)

  sub2 = rospy.Subscriber('~pose_offset_node_params', Vector3, wkr.param_cb, queue_size=1)
  
  rospy.spin()

if __name__ == '__main__':
  try:
      run()
  except rospy.ROSInterruptException:
      pass