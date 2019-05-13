CARMA Platform Release Notes
----------------------------

Version 2.8.5, released 14 May 2019 
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
