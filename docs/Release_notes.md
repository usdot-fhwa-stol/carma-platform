CARMA Platform Release Notes
----------------------------

Version 3.3.0, released February 5th, 2020
--------------------------------------------------------

**Summary:**
CARMAPlatform release version 3.3.0 is comprised of three key enhancements. First, the plugin manager is now able to return a list of services that match a user-requested capability. Second, there are new configurations for the Freightliner Cascadia, including lidar configurations which leverage the autoware points_concat_filter node to combine the two lidar devices' data into a single stream. Third, a trailer angle sensor has been added to the Freightliner Cascadia. This sensor creates a more realistic model of the truck, by providing the angle between the trailer and cab's center lines.

Enhancements in this release:
- Issue 478: Capability Interface Implementation
- Issue 489: Create configuration of carma-platform capable of operating on the freightliner cascadia using two lidars
- Issue 518: Add trailer angle sensor to CARMA trucks

Fixes in this release:
- Issue 333: There is a behavior discrepancy between original autoware and CARMA pre-release
- Issue 488: CARMAGarminLidarLiteV3DriverWrapper does not have functional docker image
- Issue 490: The libproj.so cannot be found by vector_map loader
- Issue 496: Update LaneLet2 to version 0.9.0
- Issue 497: PACMOD and/or SSC module status messages appear to have different semantics on truck
- Issue 498: The Velodyne driver registration was not functional on the truck
- Issue 499: Trailer angle sensor devices are not mapped on the host or in docker*
- Issue 506: The carma_build script is broken on autoware.ai fork v1.13
- Issue 514: Health Monitor param mismatch
- Issue 517: Load vector map to be loaded when carma is launched

*not tested on truck as of 2/5/2020

Version 3.2.0, released December 23rd, 2019
--------------------------------------------------------

**Summary:**
CARMAPlatform release version 3.2.0 includes the following four major updates. An initial converter has been developed to convert OpenDrive maps to LaneLet2's OSM format. The converter currently only covers lane geometry only. Autoware v1.13 pre-release has been integrated with CARMAPlatform. A guidance plan delegator has been developed to notify strategic plugins that the arbitrator has selected their maneuver plan, and request the corresponding trajectory plan for said plugin. Finally, a guidance re-engage capability has been added to the platform, which will allow for multiple runs to be made without restarting the software.  

Enhancements in this release:  
- Issue 416: Update Docker to use Autoware 1.12 build
- Issue 419: Add support for Lanelet2 to Docker images and upgrade to Autoware v1.13 pre-release
- Issue 431: Develop OpenDrive to LaneLet2 Converter for Geometry
- Issue 456: Create initial vehicle model
- Issue 457: Add example vehicle calibration folder for CARMA users

Fixes in this release:  
- Issue 342: Fix UI, version number not showing up
- Issue 378: Fix the Operator Override state for SSC Wrapper and Guidance
- Issue 433: Autoware plugin publishes plugin discovery topic only once
- Issue 436: Arbitrator Capabilities Interface is not initialized
- Issue 437: Autoware Plugin discovery message does not contain plugin type
- Issue 439: Abritrator node initialization function is called multiple times during INITIAL state
- Issue 443: Autoware maneuver plan does not contain planner name
- Issue 446: Arbitrator does not load plugin priorities correctly
- Issue 450: Arbitrator public node handle is initialized with namespace
- Issue 451: Arbitrator node publishes maneuver plan every 10 seconds
- Issue 452: Pure pursuit wrapper does not convert trajectory into waypoints under new CARMA planning stack


Version 3.1.0, released 18 October 2019 
--------------------------------------------------------

**Summary:**
CARMAPlatform Skyline release version 3.1.0 main highlight is the new Docker configuration and deployment for all the repositories for CARMA3. The images are now available in DockerHub under the organization name of "usdotfhwastol". Docker provides better management of library dependencies, ease of deployment and scalability. Other highlights of this release are the new configurations for the 3 different controllers such as PACMod, NewEagle and DataSpeed, the basic vehicle kinematic model, the updates to the guidance state machine, the updates to the GNSS to map converter, and the new functionality to auto select between GNSS and NDT pose.  

Below are the highlights of the issues and pull requests (PRs) that have been addressed in this release.  

- Issue 275: Novatel SPAN PwrPak7 and IMU-IGM-S1 report bad location when vehicle is moving on Lexus
- Issue 309: Remove CARMA2 java packages that no longer applies to CARMA3
- Issue 316: Update repo for conform with parameter standards
- Issue 319: Resolve Blue Lexus shifter issue
- Issue 322: Pacifica deviates from waypoints when running Autoware waypoint following
- Issue 337: Data frequency changes in Docker 
- Issue 343: Fix UI - Route Name not showing
- Issue 348: Autoware plugin does not support plugin discovery
- Issue 349: Guidance state machine does not transition properly from active to engaged on Lexus
- Issue 352: CARMA launch file does not provide placeholder to pass map cell paths 
- Issue 358: CI build fails due to missing ros-kinetic-swri-serial-util pkg
- Issue 368: Unit test fails in GuidanceStateMachine
- Issue 370: CARMASscInterfaceWrapper Docker image build failing due to dbw_mkz_msgs 
- Issue 375: SonarCloud does not report code coverage correctly
- Issue 381: Fusion CAN topic names mismatch
- Issue 385: Automatically launch RVIZ configuration file

**Repository: CARMAPlatform**  
- PR 325: Initial implementation of CARMA3 guidance node
- PR 327: Fix environment variables in launch files.
- PR 329: Fix argument name for ray_ground_filter 
- PR 330: Remove utility packages and update checkout.sh to depend on CARMAUtils
- PR 331: Updating CARMAUtils repo name
- PR 322: GNSS/NDT auto selector
- PR 338: Add enhanced warning flag output to catkin build process
- PR 339: Update Circle CI for CARMA3
- PR 371: Update test cases for guidance node
- PR 374: Created a new Turner Fairbank waypoints

**Repository: autoware.ai**  
- PR 12: Build new Autoware dockerization system on top of new carma-base
- PR 17: Add calibration file for ray_ground_filter node
- PR 18: Update ndt_matching.cpp 
- PR 22: Disable genjava in Docker build as it causes instability 

**Repository: CARMABase**  
- PR 7:  Add Autoware dependencies and Sonar Scanner
- PR 8:  Add code coverage scripts to Docker image
- PR 10: Update init-env.sh to have proper Autoware install location
- PR 11: Update SSC Dependencies
- PR 12: Update for Component Release 3.1.0
- PR 13: Update package version to 3.1.0 

**Repository: CARMAWebUI**  
- PR 17: Fix Docker image name 
- PR 18: Update CircleCI For CARMA 3

**Repository: CARMAUtils**  
- PR 14: Add carma_utils and wgs84_utils packages to repo 
- PR 15: Update checkout.sh script and CARMAUtils repo name
- PR 16: Add uncertainty_tools package
- PR 17: Update CircleCI For CARMA 3
- PR 18: Feature/enhanced build warnings 

**Repository: CARMAMsgs**  
- PR 17: Update CircleCI for CARMA 3
- PR 18: Update Plugin.msg 

**Repository: CARMAConfig** 
- PR 8: Create initial ford fusion configuration
- PR 9: Update development config to match parameter standards 
- PR 10: Remove outdated IMU config in drivers.launch 
- PR 11: Fix Lexus can configuration
- PR 12: Add state machine type
- PR 13: Fix/pacifica configuration

**Repository: CARMAVehicleCalibration** 
- PR 1: Add initial development and Lexus calibration files
- PR 2: Add vehicle calibration data
- PR 3: Update Pacifica yaw offset
- PR 5: Update blue Lexus LiDAR calibration
- PR 6: Remove duplicated folders from blue Lexus calibration

**Repository: CARMASscInterfaceWrapper** 
- PR 13: Update for Docker 
- PR 14: Use ssc module state for controller state
- PR 16: Remove PACMod dependency for controller wrapper
- PR 17: Update to match parameter standards
- PR 19: Add SSC binaries to repo
- PR 20: Add topic remapping for SSC
- PR 21: Updating checkout.sh script and CARMAUtils repo name
- PR 25: Fix Lexus configuration 
- PR 26: Update kvaser 
- PR 28: Fix Pacifica config

**Repository: CARMAVehicleModelFramework** 
- PR 8: Add -Wall flag to C/C++ build args
- PR 9: Update for kinematic model and unit testing changes

**Repository: CARMAVelodyneLidarDriver** 
- PR 11: Update for Docker 
- PR 12: Update driver to match new parameter standards
- PR 13: Fix topic names in LiDAR launch file
- PR 15: Update carma3 circle 
- PR 16: Add -Wall flag to C/C++ build args 
- PR 19: Update Docker file version numbers to target next release

**Repository: CARMANovatelGpsDriver** 
- PR 18: Update for Docker
- PR 19: Update to match parameter standards"
- PR 20: Fix dependency linkage in Docker file
- PR 22: Refactor nodelet to have carma wrapper logic in separate nodelet
- PR 23: Sync with Swri master 
- PR 24: Add publishing for INSPVAX logs to driver 
- PR 27: Add -Wall flag to C++ and C compiler flags
- PR 29: Feature/update carma3 circle 
- PR 30: Update docker version numbers to target next release

**Repository: CARMAAvtVimbaDriver** 
- PR 11: Update Docker file to match parameter standards
- PR 13: CircleCI For CARMA3
- PR 14: Add -Wall C++ and C compiler flag

**Repository: CARMADelphiEsrDriver** 
- PR 13: CircleCI For CARMA3 
- PR 14: Add -Wall flag to C/C++ build args 

**Repository: CARMADelphiSrr2Driver** 
- PR 16: Update driver to match parameter standards 
- PR 18: Update CircleCI For CARMA 3
- PR 19: Add -Wall flag to C/C++ build args

Pre-Release Version 3.0.0, released 15 July 2019 
----------------------------------- 

**Summary:**
CARMAPlatform pre-release version 3.0.0 is the first step to integrating Autoware and its components, specifically NDT matching and pure pursuit. CARMAPlatform now includes both lateral (steering) control and longitudinal (speed) control for full SAE level 2 autonomy. GNSS initialization of NDT matching has been added in order to localize the vehicle’s position on the 3D Point Cloud Map with LiDAR scan.  A temporary UI integration has been included for minimum viable functionality while awaiting further development of CARMA guidance node. Other highlights of this release are the new drivers (e.g. Velodyne LiDAR, Novatel GPS), conforming to the new CARMA3 API, Docker updates, and adding code coverage metrics to Sonar Cloud.  
**Repository: CARMAPlatform**      
-PR 303: Add missing package to build script
-PR 293: Update unit tests to match what expected in the actual code.  
-PR 288: Fixes several issues encountered during integration testing for CARMA3 beta release.  
-PR 287: Fixes topic re-mappings for the voxel grid filter after the ray_ground_filter was added. Also adds the ssc_interface (as package) into the carma_build script.  
-PR 286: Update namespace in UI launch file.  
-PR 285: Add state tracking logic to ui_integration to facilitate the status reporting of the button on the Web UI.  
-PR 284: Remap Autoware state topic to avoid conflict with CARMA guidance state topic.  
-PR 283: Now that the UI is using static topics instead of going through the interface manager this PR properly remaps those topics to their actual location.  
-PR 282: Fix UI Integration node to properly fill out the set_guidance_active response based on new guidance state.  
-PR 281: Change localization configuration for points_downsample and ndt_matching.  
-PR 280: Make the robot_status callback in interface manager configurable.  
-PR 279: Add Autoware waypoints as an example.    
-PR 278: Fix namespace issue for route generator parameters in launch file.  
-PR 277: Adds a temporary UI integration for minimum viable functionality pending further development of a real guidance node compatible with CARMA3.  
-PR 276: Add two launch files for launching CARMA3 planning stack and control stack.  
-PR 274: Adds GNSS initialization of NDT to CARMA.  
-PR 272: Fix ECEF unit test quaternion usage.  
-PR 271: This is the initial implementation of Autoware plugin. This plugin takes in a list of waypoints from Autoware and convert them into a list of evenly spaced trajectory points.  
-PR 270: Fix usage of CARMANodeHandle exceptions and compilation errors.   
-PR 267: Provides similar API as original CARMA2 route node. It can work with CARMA2 UI and let user pick the route file (waypoint csv file) to load at run time.  
-PR 266: Resolves a circular build error where functions could be included multiple times if CARMANodeHandle.h was included in multiple files in a single executable.  
-PR 260: Add CARMANodeHandle to provide exception handling.   
-PR 258: Resolves issues #252 and #253. There was a bad comment in the TF wrapper and a missing message dependency in the pure_pursuit_wrapper.  
-PR 257: Update sensor fusion CMakelists.txt file to export the wgs84_utils library so that other ROS packages can use it. Additionally, copy over the ecef_to_geodesic function from carmajava geometry package.  
-PR 255: Add a script (actually a unit test) to find out the transform between MAP and ECEF based on current lat/lon and pose in MAP frame.  
-PR 254: Refactoring the Docker versioning and image dependencies.  
-PR 251: Add map tools for splitting up PCD files larger than 1 GB.  
-PR 250: Add pure_pursuit_wrapper node. This feature enables CARMA Guidance to communicate with Autoware pure_pursuit node.  
-PR 249: Contains a node to integrate NDT matching node from Autoware.  
-PR 247: Performs and initial overhaul of CARMA2 code to make it conform to the new CARMA 3 driver API and integrate with Autoware components, specifically NDT matching.   
**Repository: CARMABase**  
-PR 1: Refactoring the Docker versioning and image dependencies.  
 **Repository: CARMASscInterfaceWrapper**    
 -PR10: Add a launch file for launching the SSC in a remappable way to this repo.  
-PR9: Make vehicle/engage topic relative in ssc_interface_wrapper.  
-PR 7: Fix topic remappings in SSC driver launch file.  
-PR 6: Corrects some mismatched topic names in the driver wrapper and updates the launch file to have correct topic re-mappings for the PACMOD.  
-PR 5: Use global report from PACMOD driver to determine the health status of the controller device; Add CAN support to include CAN messages that PACMOD provided.  
-PR 4: Add launch file for full driver.   
-PR 2: Update driver type in DriverStatus message to match CARMA3 specifications.  
-PR 1: Add initial wrapper.   
**Repository: CARMAVehicleModelFramework**  
-PR 5: Correct some dependencies in vehicle model user examples.  
-PR 4: Add support for code coverage metrics to Sonar Cloud.  
-PR 3: Implementation of dynamic vehicle model.  
-PR 1 and 2: Initial commit of vehicle model framework.   
**Repository: CARMAVelodyneLidarDriver**  
-PR 8: Fixes the topic names provided by the wrapper to match the CARMA Driver API.  
-PR 7: Make topic name relative in wrapper.  
-PR 5: Adds a Lexus ready launch file to the LIDAR driver.  
-PR 4: Updates driver type to support CARMA3 driver types defined in CARMAMsgs.  
-PR 3: Disable Sonar test reports.  
-PR 2: Add Driver wrapper.  
-PR 1: Add Sonar and Circle CI config files.  
**Repository: CARMAConfig**  
-PR 5: Update carma.launch to use single map file.  
-PR 3: Add initial Pacifica configuration folder.  
-PR 2: Updates the urdf and drivers.launch file of the Lexus to include the frames needed for heading computations needed for GNSS initialization of NDT  
-PR 1: Refactoring the Docker versioning and image dependencies.  
**Repository: CARMACohdaDsrcDriver**  
-PR 11: Fix global topic remapping in this driver.   
-PR 9: Refactoring the Docker versioning and image dependencies.  
-PR 7: Add support for code coverage metrics to Circle CI and comments for Sonar Cloud once unit tests are added.  
-PR 6: Fix comments.  
-PR 5: Setup Sonar Cloud in Circle CI.  
-PR 4: Update driver API to use global namespace in topic names.  
-PR 3: Configure Docker scripts and others for usage with new dockerized deployment to vehicle via DockerHub.  
-PR 2: Update CI file to use new Docker image.  
-PR 1: Setup Circle CI.  
**Repository: CARMAConfig**  
-PR14: Updates to SetActiveRoute.srv to add error code.   
-PR 13: Updates the DriverStatus message to support the new driver types defined for CARMA 3.  
-PR 12: Updates to DriverStatus.msg to add gps and imu.   
-PR 10: Update version ID for cav_msgs.  
-PR 8: Resolves build order issue with cav_msgs and j2735_msgs.  
-PR 7: Adds the ROS messages necessary to support an initial implementation of the CARMA Planning Plugin API.  
-PR 6: Create TrajectoryExecutionStatus.msg to add new feedback msg for control plugins.  
-PR 5: Add new messages for trajectory planning.  
-PR 4: Update DriverStatus message.  
-PR 3: Update Docker image version.  
-PR 2: Update copyrights.
-PR 1: Setup Circle CI.  
**Repository: CARMAWebUi**  
-PR 13: Add a copy of the cruising widget to be usable with the Autoware plugin.  
-PR 12: Remove IM for controller topics.  
-PR 11: Refactoring the Docker versioning and image dependencies.  
**Repository: CARMAUtils**  
-PR 11: Update driver types for CARMA3.  
-PR 9: Add code coverage metrics to Sonar Cloud.  
-PR 8: Update comments.  
-PR 7: Add Sonar Cloud to Circle CI.  
-PR 6: Update driver API for XGV controller.  
-PR 5: Update Docker image version for Circle CI.  
-PR 4: Updated driver_wrapper to make spin rate visible.  
-PR 3: Updated README file.  
-PR 2: Add driver wrapper base class.  
-PR 1: Setup Circle CI.  
**Repository: CARMACadillacSrx2013CanDriver (Private)**  
-PR 5: Update driver type for CARMA3.  
-PR 4: Apply CARMA dockerization config.   
-PR 3: Update Driver API.  
-PR 2: Update Docker version.  
-PR 1: Setup Circle CI.  
**Repository: CARMACadillacSrx2013ControllerDriver**  
-PR 12: Refactoring the Docker versioning and image dependencies.  
-PR 11: Update driver type for CARMA3.  
-PR 9: Add code coverage metrics to Sonar Cloud  
-PR 8: Add a new topic for light bar status based on front light bar.  
-PR 7: Update comment.  
-PR 6: Add Sonar Cloud support to driver.   
-PR 5: Change from private namespace to global namespace.  
-PR 4: Apply CARMA dockerization config.   
-PR 3: Update Docker image version.  
-PR 2: Setup Circle CI.  
-PR 1: Update driver to allow light bar to remain on when robotic is off.   
**Repository: CARMACadillacSrx2013ObjectsDriver (Private)**  
-PR 5: Update driver type for CARMA3.  
-PR 3: Apply CARMA dockerization config.  
-PR 2: Update Docker image version.  
-PR 1: Setup Circle CI.  
**Repository: CARMADelphiEsrDriver**  
-PR 9: Refactoring the Docker versioning and image dependencies.  
-PR 8: Update driver type for CARMA3.  
-PR 6: Add code coverage metrics to Sonar Cloud.  
-PR 5: Update comment.  
-PR 4: Add Sonar Cloud to Circle CI.    
-PR 3: Apply CARMA dockerization config.  
-PR 2: Update Docker image version.  
-PR 1: Setup Circle CI.  
**Repository: CARMADelphiSrr2Driver**  
-PR 13: Refactoring the Docker versioning and image dependencies.  
-PR 12: Update driver type for CARMA3.  
-PR 11: Add code coverage metrics to Sonar Cloud.  
-PR 10: Update comment.   
-PR 9: Add Sonar Cloud to Circle CI.  
-PR 8: Add AStuff srr2 driver to Docker file.  
-PR 7: Fix Docker image name.   
-PR 6, Apply CARMA dockerization config.  
-PR 5: Add a timeout for local messages and corrected initial driver status.   
-PR 4: Update Docker image to newest version.  
-PR 3: Update SRR2 driver wrapper.   
-PR 2: Add driver wrapper skeleton code.  
-PR 1: Setup Circle CI.  
**Repository: CARMAFreightliner2012CanDriver (Private)**  
-PR 7: Update driver type for CARMA3.  
-PR 5: Apply CARMA dockerization config.  
-PR 4: Update driver API.  
-PR 3: Update Docker image to newest version.  
-PR 1: Setup Circle CI.  
**Repository: CARMAFreightliner2012ControllerDriver**  
-PR 10: Refactoring the Docker versioning and image dependencies.  
-PR 9: Update driver type for CARMA3.  
-PR 7: Add code coverage metrics to Sonar Cloud.  
-PR 6: Update comment.   
-PR 5: Add Sonar Cloud to Circle CI.  
-PR 4: Apply CARMA dockerization config.  
-PR 3: Update driver API.  
-PR 2: Update Docker image to newest version.  
-PR 1: Setup Circle CI.  
**Repository: CARMATorcXgvControllerDriver**  
-PR 8: Refactoring the Docker versioning and image dependencies.  
-PR 7: Update driver type for CARMA3.  
-PR 5: Add code coverage metrics to Sonar Cloud.  
-PR 4: Add Sonar Cloud to Circle CI.  
-PR 3: Apply CARMA dockerization config.  
-PR 2: Update Docker image to newest version.  
-PR 1: Setup Circle CI.  
**Repository: CARMATorcPinpointDriver**  
-PR 11: Refactoring the Docker versioning and image dependencies.  
-PR 10: Update driver type for CARMA3.  
-PR 8: Add code coverage metrics to Sonar Cloud.  
-PR 7: Update comment.  
-PR 6: Add Sonar Cloud to Circle CI.  
-PR 5: Apply CARMA dockerization config.  
-PR 4: Update driver API.  
-PR 3: Update Docker image to newest version.  
-PR 2: Updated copyright.  
-PR 1: Setup Circle CI.  
**Repository: CARMANovatelGpsDriver (Forked)**  
-PR 15: Update the driver launch file to publish heading messages by default.  
-PR 14: Add dual antenna heading msg, unit test and documentation.  
-PR 10: Merge latest SWRI master repo changes.   
-PR 9: Add node name to status message.  
-PR 8: Add support to DUALANTENNAHEADING message type.    
-PR 7: Add support for HEADING2 message type.   
-PR 6: Include the addition of BESTXYZ pushed to the SWRI master.  
-PR 5: Merge latest SWRI master repo changes.   
-PR 4: Adds a Lexus ready launch file to the repo for launching the carma3 compatible driver.  
-PR 3: Update driver type for CARMA3.  
-PR 2: The SWRI robotics Novatel driver code has been modified to add CARMA system alert and driver discovery features  
-PR 1: Fix build order.  
**Repository: CARMAAvtVimbaDriver (Forked)**  
-PR 8: Refactoring the Docker versioning and image dependencies.  
-PR 4: Add Sonar Cloud and Circle CI support to repo.  
-PR 3: Add build dependencies.  
-PR 2: Update with driver status and alert.  
-PR 1: Apply CARMA dockerization config.  
**Repository: autoware.ai (Forked)**  
-PR 9 Add new launch file to voxel_grid_filter to allow remapping.  
-PR 8: Use demo map file as default transform.  
-PR 7: Make map_1_origin private and add update_rate to params file.  
-PR 6: Updates the points map loader to load map cells directly from arealist.txt file when no additional PCD paths are provided.  
-PR 5: Mark modifications on files.  
-PR 4: Update map origin.  
-PR 3: Add the feature to enable waypoint loader to load new route file based on a subscribed topic.  
-PR 2: Add ECEF map TF broadcaster.  
-PR 1: Adds the deadreckoner node from the AStuff fork of Autoware.  

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
