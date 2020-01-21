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
import unittest
import rospy
import rostest

# Imports for this test
from cav_srvs.srv import *
from cav_msgs.msg import *

# Python Unit Test class which will hold the integration test
class TestDSRCDriver(unittest.TestCase):
    
    # This is to test if DSRC driver can be found by InterfaceMgr
    def test_find_dsrc_driver(self):
        
        rospy.init_node('dsrc_test_node', anonymous=True)
        
        try:
            rospy.wait_for_service('get_drivers_with_capabilities', timeout=20)
        except rospy.ROSException, e:
            self.fail('Service get_drivers_with_capabilities not found: %s' % e)
        
        #Wait for DRIVERS_READY to call the service
        rospy.sleep(20)
        
        try:
            get_driver = rospy.ServiceProxy('get_drivers_with_capabilities', GetDriversWithCapabilities)
            get_driver_response = get_driver(['inbound_binary_msg', 'outbound_binary_msg', 'send'])
            self.assertEqual(len(get_driver_response.driver_data), 3, 'Get_driver response has wrong number of results')
            self.assertEqual(get_driver_response.driver_data[0], '/carma/drivers/dsrc/comms/inbound_binary_msg', 'inbound_binary_msg not found')
            self.assertEqual(get_driver_response.driver_data[1], '/carma/drivers/dsrc/comms/outbound_binary_msg', 'outbound_binary_msg not found')
            self.assertEqual(get_driver_response.driver_data[2], '/carma/drivers/dsrc/comms/send', 'send service not found')
        except rospy.ServiceException, e:
            self.fail('Service get_drivers_with_capabilities call failed: %s' % e)
            
    def test_send_service(self):
        
        try:
            rospy.wait_for_service('/carma/drivers/dsrc/comms/send', timeout=20)
        except rospy.ROSException, e:
            self.fail('Service sendMessage not found: %s' % e)
            
        try:
            send_message = rospy.ServiceProxy('/carma/drivers/dsrc/comms/send', SendMessage)
            send_message_response = send_message(ByteArray(std_msgs.msg.Header(), 'BSM', [1, 1, 1, 1]))
            self.assertEqual(send_message_response.errorStatus, 0, 'Service sendMessage got failure')
        except rospy.ServiceException, e:
            self.fail('Service sendMessage call failed: %s' % e)

# Main (entry point)
if __name__ == '__main__':

    PKG = 'carma'  # Package name
    rostest.rosrun(PKG, 'dsrc_driver_test', TestDSRCDriver)  # Run test