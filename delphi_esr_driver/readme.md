Objects API ( inherits ROS Driver Capabilities )
-------------------
### Parameters
~use_velocities (bool, default: false)
: should this node query the interface manager for a position/velocity driver and send these velocities
: to the delphi_radar

~radius_curvature (int32_t, default: 8191)
: radius of curvature of the fascia covering the radar

~lateral_mounting_offset (float, default: 0.0)
: mounting offset in meters to the right from driver's perspective

~angle_misalignment (float, default: 0.0)
: angle offset of the radar with degrees clockwise to the right of the front of the vehicle

~mmr_upside_down (bool, default: false)
: indicates whether the radar is upside down

~blockage_disable (bool, default: false)
: used to disable blockage during certain scenarios

~use_angle_misalignment (bool, default: false)
: flag to allow controller to initialize the auto alignment angle

~lr_transmit (bool, default: true)
: set to true if the radar should use LR Mode

~mr_transmit (bool, default: true)
: set to true if the radar should use MR Mode

~maximum_tracks (int32_t, default: 64)
: max number of objects of interest to report

~high_yaw_angle (int32_t, default: 0)
: Angle misalignment used for applications where the radar is mounted at very high yaw angles

~raw_data_enable (int32_t : enum, default: Filtered)
: Raw data Enable (0-Filtered, 1-RawData)

~grouping_mode (int32_t : enum, default: None)
: Grouping mode (0-No grouping, 1-Moving targets, 2-stationary targets, 3-both)

~sensor_frame (string, default: "delphi")
: frame corresponding to the sensor

~device_name (string, default: "can0" )
: name of the socketcan device corresponding to the CAN bus communicating with the radar


### Published Topics
~sensor/objects (cav_msgs/ExternalObjectList)
> Objects sensed by this sensor are published to this topic in the frame ~sensor_frame

/diagnostics (diagnostic_msgs/DiagnosticArray)
> This driver uses the diagnostic_updater API to publish diagnostic statuses to the /diagnostic topic.
