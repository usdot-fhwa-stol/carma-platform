# RobotEnabled.msg

# This message is feedback to the system to inform the ROS network that 
# the SRX controller has enabled robotic control. Also contains feedback for 
# injected torque and braking.

# True if the controller is acvtively controlling the vehicle
bool robot_enabled

# The value of Torque being commanded to the vehicle Nm
float64 torque

# Braking deceleration - The commanded braking deceleration m/s^2
float64 brake_decel

