#!/usr/bin/env python
# import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import unittest
import rospy

# Because of transformations
import tf

import tf2_ros
import geometry_msgs.msg
import math
from cav_srvs.srv import *


# A sample python unit test
class TestTransformServer(unittest.TestCase):
    # Helper function to compare two transform messages
    def is_equal_transform(self, msg1, msg2):
        tf1 = msg1.transform
        tf2 = msg2.transform
        return msg1.header.frame_id == msg2.header.frame_id and \
               msg1.child_frame_id == msg2.child_frame_id and \
               math.abs(tf1.translation.x - tf2.translation.x) < 0.0000001 and \
               math.abs(tf1.rotation.x - tf2.rotation.x) < 0.0000001 and \
               math.abs(tf1.rotation.y - tf2.rotation.y) < 0.0000001 and \
               math.abs(tf1.rotation.z - tf2.rotation.z) < 0.0000001 and \
               math.abs(tf1.rotation.w - tf2.rotation.w) < 0.0000001

    # Test if the get transform service is working
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
        tf_stamped.header.frame_id = "map"
        tf_stamped.child_frame_id = "child_frame"

        trans = [1.0, 1.0, 1.0]
        tf_stamped.transform.translation.x = trans[0]
        tf_stamped.transform.translation.y = trans[1]
        tf_stamped.transform.translation.z = trans[2]

        euler = [0, 0, math.radians(45)]
        q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        tf_stamped.transform.rotation.x = q[0]
        tf_stamped.transform.rotation.y = q[1]
        tf_stamped.transform.rotation.z = q[2]
        tf_stamped.transform.rotation.w = q[3]

        tfMat = tf.transformations.compose_matrix(angles=euler, translate=trans)

        # Build inverse transform
        inverseMat = tf.transformations.inverse_matrix(tfMat)
        scale, shear, angles, translate, perspective = tf.transformations.decompose_matrix(inverseMat)
        inverseTF = geometry_msgs.msg.TransformStamped()
        inverseTF.header.frame_id = "child_frame"
        inverseTF.child_frame_id = "map"

        inverseTF.transform.translation.x = translate[0]
        inverseTF.transform.translation.y = translate[1]
        inverseTF.transform.translation.z = translate[2]

        q_inv = tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
        inverseTF.transform.rotation.x = q_inv[0]
        inverseTF.transform.rotation.y = q_inv[1]
        inverseTF.transform.rotation.z = q_inv[2]
        inverseTF.transform.rotation.w = q_inv[3]

        # Publish transform using tf2 transform broadcaster
        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(tf_stamped)

        rospy.sleep(5)  # Provide time for TransformServer to receive published transform

        # Lookup regular transform using service
        try:
            get_transform = rospy.ServiceProxy('get_transform', GetTransform)
            response = get_transform("map", "child_frame")
            if response.errorStatus != 0:
                self.fail("Response returned with error %s" % response.errorStatus)
            rospy.loginfo("\n\nResponse = " + str(response) + "\n\n")
            self.assertTrue(self.is_equal_transform(response.transform, tf_stamped.transform), "Returned transform is not equal")
        except rospy.ServiceException, e:
            self.fail("Service call failed: %s" % e)

        # Lookup inverse transform using service
        try:
            get_transform = rospy.ServiceProxy('get_transform', GetTransform)
            response = get_transform("child_frame", "map")
            rospy.loginfo("\n\nInvResponse = " + str(response) + "\n\n")
            if response.errorStatus != 0:
                self.fail("Response returned with error %s" % response.errorStatus)
            self.assertTrue(self.is_equal_transform(response.transform, inverseTF), "Returned transform is not equal")
        except rospy.ServiceException, e:
            self.fail("Service call failed: %s" % e)

        # If we get this far then the test has passed

if __name__ == '__main__':
    import rostest

    PKG = 'carma'
    rostest.rosrun(PKG, 'test_transform_server', TestTransformServer)
    #rospy.Publisher('hello', String, queue_size=10)
    #     import rospy
    # from std_msgs.msg import String
    #
    # def talker():
    #     pub = rospy.Publisher('chatter', String, queue_size=10)
    #     rospy.init_node('talker', anonymous=True)
    #     rate = rospy.Rate(10) # 10hz
    #     while not rospy.is_shutdown():
    #         hello_str = "hello world %s" % rospy.get_time()
    #         rospy.loginfo(hello_str)
    #         pub.publish(hello_str)
    #         rate.sleep()
    #
    # if __name__ == '__main__':
    #     try:
    #         talker()
    #     except rospy.ROSInterruptException:
    #         pass
