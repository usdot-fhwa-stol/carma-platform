CARMA Platform Release Notes
----------------------------

Version 3.0.0, released 15 July 2019 
----------------------------------- 
**Repository: CARMAPlatform** 
-PR 260 Adds a carma_utils package containing CARMANodeHandle.h/.cpp files.
-PR 257 Updates sensor fusion CMakelists.txt file to export the wgs84_utils library
-PR 254 Refactoring the docker versioning and image dependencies
-PR 251 Add map tools for splitting up pcd files
-PR 250 Add pure_pursuit_wrapper node.
-PR 249 Contains a node to integrate NDT matching node from Autoware.
-PR 247 Performs and initial overhaul of CARMA2 code to make it conform to the new CARMA 3 driver API and integrate with Autoware components
**Repository: CARMASscInterfaceWrapper**
-PR 8 Launching the driver launch file for this wrapper does not map
-PR 3 Make launch file capable of launching full driver
**Repository: CARMANovatelGpsDriver**
-PR 15, Publish heading messages when launching driver
-PR 14, Feature/add dual antenna heading msg
-PR 9, Add node name to status message
-PR 7, Feature/add header2 msg
-PR 6, Add support for BESTXYZ topic
-PR 4, Add launch file for full driver
-PR 3, Update driver type for carma3
-PR 2, The swri robotics novatel driver code has been modified to add CARMA system alert and driver discovery features
-PR 1, Add fix for messages build order
**Repository: CARMAVelodyneLidarDriver**
-PR 8, fixes the topic names provided by the wrapper to match the CARMA Driver API
-PR 7, make topic name relative in wrapper
-PR 5, adds a lexus ready launch file to for the lidar driver.
-PR 4, updates driver type to support carma 3 driver types defined in CARMAMsgs
-PR 2, add Driver wrapper
**Repository: autoware.ai**
-PR 9 Add new launch file to voxel_grid_filter to allow remapping
-PR 8 Use demo map file as default transform
-PR 7 Make map_1_origin private and add update rate to params file
-PR 6 Updates the points map loader to load map cells directly from arealist.txt file when no additional pcd paths are provided.
-PR 3 Add the feature to enable waypoint loader to load new route file based on a subscribed topic.
-PR 1 Adds the deadreckoner node from the AStuff fork of autoware.
**Repository: CARMAConfig**
-PR 5, update carma.launch to use single map file
-PR 3, Add initial pacifica config folder
-PR 2, Update urdf file and drivers.launch for heading support
-PR 1, Feature/docker refactor
**Repository: CARMACohdaDsrcDriver**
-PR 11, Fix/node topic remapping
-PR 9, Feature/docker refactor
-PR 7, Add code coverage metrics to sonar cloud
-PR 4, Update driver API
-PR 3, Feature/dockerization
-PR 2, Update CI file to use new docker image
**Repository: CARMAMsgs**
-PR 13, Update driver type for carma3
-PR 8, Fix cav_msgs cmakelists and package xml for j2735 deps 
-PR 7, Planning Plugin API 
-PR 6, Create TrajectoryExecutionStatus.msg 
-PR 5, Feature/messages for planning
-PR 4, Update driver status msg
-PR 3, Update docker version
**Repository: CARMATorcPinpointDriver**
-PR 11, Feature/docker refactor
-PR 10, Update driver type for carma3
-PR 8, Add code coverage metrics to sonar cloud
-PR 5, Apply CARMA dockerization config
-PR 3, Update docker version
**Repository: CARMACadillacSrx2013ControllerDriver**
-PR 12, Feature/docker refactor
-PR 11, Update driver type for carma3
-PR 9, Add code coverage metrics to sonar cloud
-PR 8, Changes to add a new topic for light bar status based on front light bar 
-PR 6, Add sonar cloud support to driver 
-PR 4, Feature/dockerization
-PR 3, Update docker image version
**Repository: CARMADelphiEsrDriver**
-PR 9, Feature/docker refactor
-PR 8, Update driver type for carma3
-PR 6, Add code coverage metrics to sonar cloud
-PR 3, Feature/dockerization
-PR 2, Update docker version
**Repository: CARMADelphiSrr2Driver**
-PR 13, Feature/docker refactor
-PR 11, Add code coverage to sonar cloud
-PR 8, Add AStuff srr2 driver to docker file
-PR 6, Apply CARMA dockerization config
-PR 4, Update docker image to newest version
-PR 2, Driver Wrapper Skeleton Code
**Repository: CARMAFreightliner2012ControllerDriver**
-PR 10, Feature/docker refactor
-PR 9, Update driver type for carma3
-PR 7, Add code coverage metrics to sonar cloud
-PR 4, Apply CARMA dockerization config
-PR 2, Update docker image version
**Repository: CARMATorcXgvControllerDriver**
-PR 7, Update driver type for carma 3
-PR 5, Add code coverage metrics to sonar cloud
-PR 3, Apply CARMA dockerization config
-PR 2, Update docker version
**Repository: CARMAWebUi**
-PR 13, Feature/autoware ui widget
-PR 12, Remove IM for controller topics
-PR 11, Feature/docker refactor
**Repository: CARMAAvtVimbaDriver**
-PR 8, Feature/docker refactor
-PR 4, Add sonarcloud and circle ci support to repo
-PR 3, Add build dependencies
-PR 2, update with driver status and alert
-PR 1, Apply CARMA dockerization config
**Repository: CARMADriverUtils**
-PR 11, update driver types for carma3
-PR 9, add code coverage metrics to sonar cloud
-PR 7, add sonar cloud to ci 
-PR 6, update driver api 
-PR 5, update docker image version for ci
-PR 4, make spin rate visible
-PR 2, driver Wrapper Base Class
**Repository: CARMAVehicleModelFramework**
-PR 5, correct some dependencies in vehicle model user examples
-PR 4, add support for code coverage metrics to sonar cloud
-PR 3, implementation of dynamic vehicle model
-PR 2, initial commit of vehicle model framework with examples
-PR 1, initial commit of vehicle model framework

Version 2.9.0, released 15 May 2019 
----------------------------------- 

-PR 199, ignore raw CAN data in rosbags  
-PR 201, fix speed limit handling  
-PR 203, security checks on mobility message strategy strings  
-PR 206, address security issues  
-PR 208, expanded docker for developer testing  
-PR 210, fix maneuver dependency in Route node  
-PR 212, added unit tests for strategy string security  
-PR 214, cleaned up old messaging  
-PR 215, refactored GuidanceCommands into separate node  
-PR 216, fixed high priority SonarCloud issues  
-PR 223, added test coverage metrics to SonarCloud  
-PR 224, fix for test coverage metrics  
-PR 228, fix for test coverage metrics  
-PR 232, fix dockerfile version metadata  
-PR 233, fix test coverage metrics  
-PR 236, add SonarCloud status reporting to README

Version 2.8.4, released 04 March 2019 
------------------------------------- 

-PR 10, add NCV handling to traffic signal plugin  
-PR 12, fix obstacle subscriber to traffic signal plugin  
-PR 21, add unit tests for NCV detection  
-PR 23, fix timestamp interpolation in traffic signal plugin  
-PR 28, fix NCV integration problems in traffic signal plugin  
-PR 29, fixes for NCV conflict detection in traffic signal plugin  
-PR 30, removed unused traffic signal plugin message listeners  
-PR 35, configured docker for CARMA deployments  
-PR 37, fix .dockerignore  
-PR 38, fix traffic signal GUI widget  
-PR 56, refactor conflict manager  
-PR 43, 61, updates to administrative docs  
-PR 65, initial integration with Circle CI  
-PR 72, GUI logo update  
-PR 76, Docker remote start  
-PR 85, tuned ACC PID parameters  
-PR 87, fix ACC trigger conditions  
-PR 89, fix race condition in sensor fusion  
-PR 91, reduced log output  
-PR 92, bypass coarse plan in traffic signal plugin  
-PR 94, fix docker scripts  
-PR 100, refactored into multiple repositories  
-PR 102, fix gradle build error  
-PR 104, allow light bar operations while vehicle is off  
-PR 106, Circle CI build integration  
-PR 108, added html test report  
-PR 170, added AutonomouStuff message specs  
-PR 171, updated copyright dates for 2019  
-PR 173, update docker image version to 2.8.2  
-PR 174, fixes broken guidance unit tests  
-PR 176, dockerhub integration  
-PR 178, remove driver connection dependence on Interface Manager  
-PR 181, Circle CI integration  
-PR 182, update image dependencies in docker  
-PR 183, fixed platooning unit test  
-PR 185, fix docker shutdown  
-PR 186, enhanced carma tool for using docker  
-PR 187, fix to Circle CI integration  
-PR 188, fix lateral control publish topic name  
-PR 189, platooning demo configuration  
-PR 191, fix docker command error  
-PR 196, added light bar indicator to UI  

Version 2.8.1, released 15 November 2018  
----------------------------------------  

-Issue #51 fixed to prevent platooning plugin from failing on startup.
-Updates to several administrative documents.

Version 2.8.0, released 31 October 2018
---------------------------------------

-Added traffic signal plugin that provides GlidePath functionality (eco-approach and
 departure at a signalized intersection) for one or more fixed phase plan traffic signals 
 in the planned route.  
-Issue 1015, fixed UI to use Interface Manager for all driver topics and sensor fusion for
 source of vehicle speed  
-Issue 1041, fixed NPE caused by end of route in VehicleAwareness  
-Issue 1031, gave plugins access to ROS time  
-Issue 1078, allowed traffic signal popup for operator confirmation to appear earlier  

Version 2.7.4, released 22 October 2018
---------------------------------------

Redacted files with sensitive data that cannot be publicly distributed.


Version 2.7.3, released 09 October 2018
---------------------------------------

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
------------------------------------

This is the first public release of the CARMA platform (internally known as Prototype I).

Installation must be performed from a development computer.  Once the system is built 
locally on that computer, the remote installer tool, found in the engineering_tools 
directory, can be run to transfer the executable and configuration files to the target 
vehicle computer in a directory named /opt/carma. Please see the User Guide in the docs
folder for more details.

Operating the software requires that it is installed in a properly modified vehicle, with
corresponding device drivers in place.  Use at FHWA Saxton Lab is on customized Cadillac SRX, 
Ford Escape and Freightliner Cascadia truck.   
