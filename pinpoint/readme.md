Controller API ( inherits ROS Driver Capabilities )
-------------------
### Parameters
~address (string, default: "192.168.88.29")
: ipv4 address for PinPoint localization server

~loc_port (string, default: "9501")
: port for localization server

~odom_frame (string, default: "odom")
: continuous local frame of robot

~base_link_frame (string, default: "base_link")
: frame referencing the base vehicle platform, typically center of the rear axel

~sensor_frame (string, default: "pinpoint")
: the coordinate frame that PinPoint uses as its reference

~publish_tf (bool, default: false)
: if set to true, this driver will publish the odom->base_link transform

### Published Topics
~/position/velocity ( geometry_msgs/TwistStamped)
> PinPoint reported velocity published in base_link frame

~/position/nav_sat_fix ( sensor_msgs/NavSatFix )
> PinPoint reported global pose

~position/odometry ( nav_msgs/Odometry )
> PinPoint reported local pose. This is the position/orientation offset of PinPoint since boot/reset, This transform is not
> zeroed at start of this driver

~position/heading ( cav_msgs/HeadingStamped )
> PinPoint reported heading in degrees east of north

/diagnostics (diagnostic_msgs/DiagnosticArray)
> This driver uses the diagnostic_updater API to publish diagnostic statuses to the /diagnostic topic.
