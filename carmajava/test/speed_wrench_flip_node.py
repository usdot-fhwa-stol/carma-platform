#!/usr/bin/env python
# license removed for brevity
#
# ROS Node converts a mobility request or path message from a bag file to have an updated time stamp
# This causes that message to appear as if it had just been received
#
import time
import rospy

from cav_msgs.msg import SpeedAccel
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

# Function to return current time
current_time_millis = lambda: long(round(time.time() * 1000))

MPS_PER_MPH = 0.44704

# Node Class
class SpeedWrenchFlipNode(object):
    def __init__(self):
        # Init nodes
        rospy.init_node('speed_wrench_flip_node', anonymous=True)

        # Params
        #self.dist_to_merge = rospy.get_param('~dist_to_merge')
        # Convert mph to mps
        #self.speed_limit = rospy.get_param('~speed_limit') * MPS_PER_MPH
        
        # Publishers
        self.cmd_speed_pub = rospy.Publisher('/srx_controller/controller/cmd_speed', SpeedAccel, queue_size=10)
        self.cmd_effort_pub = rospy.Publisher('/srx_controller/controller/cmd_longitudinal_effort', Float32, queue_size=10)

        # Subscribers
        self.switch_pub = rospy.Subscriber('/switch', Float32MultiArray, self.switch_cb)

        self.useWrenchEffort = True
        self.wrenchEffortValue = -100.0
        self.speedValue = 5
        self.maxAccel = 2.5
        
        #Spin Rate
        self.spin_rate = rospy.Rate(10)

        self.lastTime = current_time_millis()


    # Function to handle request messages from bag file
    def switch_cb(self, cmd_msg):
        if (cmd_msg.data[0] < 0): 
            self.wrenchEffortValue = cmd_msg.data[1]
            print("Effort: " + str(self.wrenchEffortValue))
            self.speedValue = cmd_msg.data[2]
            self.maxAccel =  cmd_msg.data[3]
            print("Speed: " + str(self.speedValue))
            print("MaxAccel: " + str(self.maxAccel))
            self.useWrenchEffort = True
        elif (cmd_msg.data[0] > 0): # Use speed command
            self.speedValue = cmd_msg.data[2]
            self.max_accel = cmd_msg.data[3]
            print("Speed: " + str(self.speedValue))
            print("MaxAccel: " + str(self.maxAccel))
            self.useWrenchEffort = False

    def spin(self):
        self.cmd_speed_pub
        self.cmd_effort_pub
        while not rospy.is_shutdown():
            if (self.useWrenchEffort):
                currentTime = current_time_millis()
                if (currentTime - self.lastTime > 3000):
                    speed_msg = SpeedAccel()
                    speed_msg.speed = self.speedValue
                    speed_msg.max_accel = self.maxAccel
                    self.cmd_speed_pub.publish(speed_msg)
                    self.lastTime = currentTime
                else:
                    effort_msg = Float32()
                    effort_msg.data = self.wrenchEffortValue
                    self.cmd_effort_pub.publish(effort_msg)
            else:
                speed_msg = SpeedAccel()
                speed_msg.speed = self.speedValue
                speed_msg.max_accel = self.maxAccel
                self.lastTime = current_time_millis()
                self.cmd_speed_pub.publish(speed_msg)
            
            self.spin_rate.sleep()

if __name__ == '__main__':
    try:
        swfn = SpeedWrenchFlipNode()
        swfn.spin()
        # prevent python from exiting until this node is stopped 
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
