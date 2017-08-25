# RobotEnabled.msg

# This message is feedback to the system to inform the ROS network that 
# the SRX controller has enabled robotic control. Also contains feedback for 
# injected torque and braking.

# true if the controller is actively commanding the vehicle
bool robot_active

#Status of robot_enabled, if set the controller is listening to command topics
bool robot_enabled

# The value of Torque being commanded to the vehicle Nm
float64 torque

# Braking deceleration - The commanded braking deceleration m/s^2
float64 brake_decel

