Controller API ( inherits ROS Driver Capabilities )
-------------------
### Parameters
~k_p ( double, default : 7.5 )
: proportional gain constant for closed loop control

~k_i ( double, default : 3 )
: integral gain constant for closed loop control

~k_d ( double, default : 0 )
: derivative gain constant for closed loop control

~enabled_at_start ( bool, default: false )
: If true robotic_enabled is set true at start, false otherwise

### Published Topics
sent_messages ( can_msgs/Frame )
> CAN frames sent to vehicle dbw module, this should be advertised by a a node communicating with CAN hardware

~control/robot_status (cav_msgs/RobotEnabled )
> publishes the Robot status of the controller driver, robot_active is set when the Truck Controller reports active control through wrench effort or speed commands, robot_enabled is set when the robotic_enabled flag is set through the enable_robotic service

### Subscribed Topics
received_messages ( can_msgs/Frame )
> We listen to this topic for messages from the dbw module

~/control/cmd_speed (cav_msgs/SpeedAccel )
> Messages received on this topic are converted to closed loop speed commands to the sent_messages topic. The units used for speed is m/s limited to a max of 31.25 (~70 mph). The units used for accel is (m/s)/s limited to 3.5.

~/control/cmd_longitudinal_effort
> Messages received on this topic are converted to open loop effort commands sent to the sent_messages topic. The effort is bounded [-100, 100].

### Services
~control/set_lights (cav_srvs/SetLights.srv)
> Set the state of the light bar

~control/get_lights (cav_srvs/GetLights.srv)
> Get the light status of the set value of the light bars. (NOTE: this may not match up with the physical lights if there is a communication disconnect)

~control/enable_robotic (cav_srvs/SetEnableRobotic.srv)
> This service allows another node to set/unset the robot_enabled flag. If the robot_enabled flag is set to false all messages received on the cmd_speed and cmd_longitudinal_effort topicss of this node are ignored. The controller will send disable robotic commands down to the hardware. The status of this robot_enabled flag is publisshed in the robot_status topic.

~control/engine_brake_ctrl (cav_srvs/EngineBrakeControl.srv)
> This service allows another node to control the engine braking command. There are three parts to the service including the mode, command, and level. In order to robotically command engine braking the mode must be set to bypass and the command set to enabled. In this mode the level can be set to low, medium, or high.


## Control Flow to command robotic

This node attempts to abstract away the requirements of the underlying hardware and reduce commanding speeds/efforts to the API specified in this document. For the sake of having this 
as a reference somewhere the following procedure is required to enter robotic mode in the 2013 Truck with the CARMA hardware.

1. Turn on Electronics
2. Turn on Ignition
3. Launch ROS software ( this should include the truck_controller_node assuming enabled_at_start is set to false) 
4. Prime the Truck 2013 ACC by accelerating > 25mph and entering ACC
5. Call the ~/control/enable_robotic service with set = true
	This will allow the truck_controller_node to forward commands received
6. Begin commanding speed/effort at greater than 5Hz ( < 200ms delay )
7. Driver must then manually enter robotic by double tapping the ACC set button.

From this point on the vehicle will remain in robotic as long as it receives commands @ >5z, the robot_enabled flag remains set to true, and there is no manual override. If a process wishes to remain in robotic but not command effort it should send cmd_longitudinal_efforts of 0% @ >5hz

 
