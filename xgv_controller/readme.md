Controller API ( inherits ROS Driver Capabilities )
-------------------
### Parameters
~subsystem_id (int, default: 1)
>The subsystem id of the XGV ByWire

~node_id (int, default: 1)
>The node id of the XGV ByWire

~motion_profile_duration_ms (int, default: 400)
>The duration the XGV will latch to a motion profile

~authority_level (int, default: 6)
>The authority level used to request control of XGV Components

~node_manager_config_file(string, default: "nodeMnaager.conf")
>This is the file holding the node manager configuration. SHould either be a relative directory from the Working Directory
>or should be an absolute path

### Published Topics
~control/robot_status (cav_msgs/RobotEnabled )
> publishes the Robot status of the controller driver, robotic_active is set when the SRX Module reports active control, robot_enabled is set when the robotic_enabled flag is set through the enable_robotic service

### Subscribed Topics
~/control/cmd_speed (cav_msgs/SpeedAccel )
> Messages received on this topic are converted to closed loop speed commands to the sent_messages topic. The units used are m/s and m/s^2.

~/control/cmd_longitudinal_effort
> Messages received on this topic are converted to open loop effort commands sent to the sent_messages topic. The effort is bounded [-100,100].

### Services
~control/enable_robotic (cav_msgs/SetEnableRobotic)
> This service allows another node to set/unset the robot_enabled flag. If the robot_enabled flag is set to false all messages received on the cmd_speed and cmd_longitudinal_effort topicss of this node are ignored. The controller will send disable robotic commands down to the hardware. The status of this robot_enabled flag is publisshed in the robot_status topic.

## Configuration

The xgv_controller_node also launches a Jaus Node Manager. This node_manager requires a configuration file found in
.../xgv_controller/etc/nodeManager.conf. This file should be edited to use either node or subsystem networking depending
on the XGV configuration. Also make sure to set the corresponding JUDP_IP_Address to the IP of the interface connected
to the XGV network.



 
