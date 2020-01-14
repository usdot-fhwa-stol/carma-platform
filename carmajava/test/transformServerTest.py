#!/usr/bin/env python
#
# Copyright (C) 2018-2020 LEIDOS.
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

# Standard imports for rostest
# There is also a roslib but it probably won't be needed
import unittest
import rospy
import rostest

# Imports for this test
import tf
import geometry_msgs.msg
from cav_srvs.srv import *


# Python Unit Test class which will hold the integration test
class TestTransformServer(unittest.TestCase):
    # Helper function to compare two transform messages
    def are_equal_transforms(self, msg1, msg2):
        tf1 = msg1.transform
        tf2 = msg2.transform
        return msg1.header.frame_id == msg2.header.frame_id and \
               msg1.child_frame_id == msg2.child_frame_id and \
               abs(tf1.translation.x - tf2.translation.x) < 0.0000001 and \
               abs(tf1.translation.y - tf2.translation.y) < 0.0000001 and \
               abs(tf1.translation.z - tf2.translation.z) < 0.0000001 and \
               abs(tf1.rotation.x - tf2.rotation.x) < 0.0000001 and \
               abs(tf1.rotation.y - tf2.rotation.y) < 0.0000001 and \
               abs(tf1.rotation.z - tf2.rotation.z) < 0.0000001 and \
               abs(tf1.rotation.w - tf2.rotation.w) < 0.0000001

    # Test if the get_transform service is working
    def test_get_transform_service(self):
        # Start the node
        rospy.init_node('test_transform_test_node', anonymous=True)
        # Wait for service to be available
        try:
            rospy.wait_for_service('get_transform', timeout=5)
        except rospy.ROSException, e:
            self.fail("Service not found: %s" % e)

        # Build transform
        tf_stamped = geometry_msgs.msg.TransformStamped()

        tf_stamped.header.stamp = rospy.Time.now()
        tf_stamped.header.frame_id = "base_link"
        tf_stamped.child_frame_id = "host_vehicle"

        # If URDF file is updated this transform will need to be changed as well
        trans = [1.524, 0.0, -0.3556]
        tf_stamped.transform.translation.x = trans[0]
        tf_stamped.transform.translation.y = trans[1]
        tf_stamped.transform.translation.z = trans[2]

        euler = [3.14159265359, 0, 0]
        q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2], 'sxyz')  # Specify fixed rpy rotation
        tf_stamped.transform.rotation.x = q[0]
        tf_stamped.transform.rotation.y = q[1]
        tf_stamped.transform.rotation.z = q[2]
        tf_stamped.transform.rotation.w = q[3]

        rospy.sleep(20)  # Provide time for Transform server to load all transforms
        # Lookup regular transform using service
        try:
            get_transform = rospy.ServiceProxy('get_transform', GetTransform)
            response = get_transform("base_link", "host_vehicle", rospy.Time())
            if response.errorStatus != 0:
                self.fail("Response returned with error %s" % response.errorStatus)
            self.assertTrue(self.are_equal_transforms(response.transform, tf_stamped), "Returned transform is not equal")
        except rospy.ServiceException, e:
            self.fail("Service call failed: %s" % e)

        # If we get this far then the test has passed
# Main (entry point)
if __name__ == '__main__':

    PKG = 'carma'  # Package name
    rostest.rosrun(PKG, 'test_transform_server', TestTransformServer)  # Run test

