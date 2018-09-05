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
bool torque_validity

# Braking deceleration - The commanded braking deceleration m/s^2
float64 brake_decel
bool brake_decel_validity

# Throttle effort as reported by vehicle [-100,100]
float64 throttle_effort
bool throttle_effort_validity

# Braking effort as reported by vehicle [-100,100]
float64 braking_effort
bool braking_effort_validity

