Controller API ( inherits ROS Driver Capabilities )
-------------------
###Parameters
~k_p ( double, default : 0.1 )
: proportional gain constant for closed loop control

~k_i ( double, default : 0.01 )
: integral gain constant for closed loop control

~k_d ( double, default : 0.005 )
: derivative gain constant for closed loop control

~enabled_at_start ( bool, default: false )
: If true robotic_enabled is set true at start, false otherwise

###Published Topics
sent_messages ( can_msgs/Frame )
: CAN frames sent to vehicle dbw module, this should be advertised by a a node communicating with CAN hardware

~control/robot_status (cav_msgs/RobotEnabled )
: publishes the Robot status of the controller driver, robotic_active is set when the SRX Module reports active control, robot_enabled is set when the robotic_enabled flag is set through the enable_robotic service

/diagnostics (diagnostic_msgs/DiagnosticArray)
: This driver uses the diagnostic_updater API to publish diagnostic statuses to the /diagnostic topic.

###Subscribed Topics
received_messages ( can_msgs/Frame )
: We listen to this topic for messages from the dbw module

~/control/cmd_speed (cav_msgs/SpeedAccel )
: Messages received on this topic are converted to closed loop speed commands to the sent_messages topic. The units used are m/s and m/s^2.

~/control/cmd_longitudinal_effort
: Messages received on this topic are converted to open loop effort commands sent to the sent_messages topic. The effort is bounded [-100,100].

###Services
~control/set_lights (cav_msgs/SetLights.srv)
: Set the state of the light bar

~control/get_lights (cav_msgs/GetLights.srv)
: Get the light status of the set value of the light bars. (NOTE: this may not match up with the physical lights if there is a communication disconnect)

~control/enable_robotic (cav_msgs/SetEnableRobotic)
: This service allows another node to set/unset the robot_enabled flag

 