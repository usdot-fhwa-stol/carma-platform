CARMA Platform Release Notes
***********************************************************************

Version 2.7.3, released 09 October 2018
***********************************************************************
Material changes to the software in this version are:

-PR 982, fixed unit tests
-PR 983, added decoding of J2735/2016 SPAT & MAP messages
-PR 988, improved platooning plugin's use of mobility message connection
-PR 1003, configuration changes for operation on the Saxton Ford Escape
-PR 1005, fix CAN driver problems for the Saxton Freightliner Cascadia
-PR 1007, allows handling of larger rosbag files
-PR 1010, allows toggling of wrench effort control
-PR 1011, renamed URDF file to be generic
-PR 1012, fixes issue #999 for sensor fusion handling of aged object data
-PR 1020, fixes issue #1017 for MAP connects-to list
-PR 1022, fixes DSRC driver config data for SPAT & MAP messages
-PR 1030, added ROS messages to pass traffic signal info to the UI
-PR 1033, fix MessageConsumer for SPAT & MAP
-PR 1035, added UI widget for traffic signal plugin
-PR 1036, allow guidance plugins to set up ROS service servers

Version 2.7.2, released 17 July 2018
-------------------------------------------------------------------------------------------
This is the first public release of the CARMA platform (internally known as Prototype I).

Installation must be performed from a development computer.  Once the system is built 
locally on that computer, the remote installer tool, found in the engineering_tools 
directory, can be run to transfer the executable and configuration files to the target 
vehicle computer in a directory named /opt/carma. Please see the User Guide in the docs
folder for more details.

Operating the software requires that it is installed in a properly modified Cadillac SRX, 
Ford Escape or Freightliner Cascadia truck with device drivers in place.  
