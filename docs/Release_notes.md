CARMA Platform Release Notes
----------------------------

Version 4.2.0, released July 29th, 2022
----------------------------------------

**Summary:**
Carma-platform release version 4.2.0 is comprised of three major enhancements. First, Cooperative Traffic Management (CTM) Speed harmonization. Second, Cooperative Lane Follow (CLF) - Platoon Formation, Operation, Dissolution. Third, Cooperative Lane Coordination (CLC), cooperative lane merge. Along with the above enhancements, several bug fixes and CI related enhancements are included in this release.

Carma Platform:

Enhancements in this release:

- Issue 1766: Updated port the Traffic Incident Parser node to ROS2 and updated several subscribers and publishers created within carma_wm_ros2 in order to support publishers being able to re-publish earlier messages to late-joining subscribers.
- Issue 1762: Updated LCI (Light Controlled Intersection) Strategic Plugin so that it uses the scheduled message from the intersection for setting its ET algorithm parameters instead of computing them based on SPAT if the message is available.
- Issue 1793: Updated Carma wm_ctrl timer classes to Ros2 and changed bridge map directions to 2 to 1 in Carma-config.
- Issue 1812: Updated Configuration parameters in Platooning Strategic IHP logic for enable/disable Front& Rear cut-in join functionality.
- Issue 1810: Added port route node to ROS2 and updated Carma environment launch files and Guidance launch files.
- Issue 1826: Added IHP Strategic and Control plugins, Also Implemented the 2nd iteration of the integrated highway prototype which include cut-in rear and front platoon joins whereas Departure from a platoon and speed harmonization is done via Carma-cloud.
Fixes in this release:

- Issue 1737: Fixed Current J2735 MAP.msg creator which does not work with TFHRC Vector Map.osm where the Center points created by the tool does not align with road center points in OSM map.
- Issue 1794: Fixed Geofence (getAffectedLaneletOrAreas) function such that Carma should not crash if it detected Geofence (getAffectedLaneletOrAreas) with only 1 point.
- Issue 1799: Fixed IHP Control plugin Cmake file to be compatible with Carma platform's Docker build process.
- Issue 1810: This issue includes small fixes after updating port the route node to ROS2 :
o Stop and Wait Plugin incorrectly had the route node as a dependency, this dependency has been removed.
o Topic naming mismatches were fixed for Trajectory Executor, Roadway Objects, and Traffic Incident Parser.
o ROS1 messages and service related to routing (SetActiveRoute.srv, RouteState.msg) had their field names updated to Snake case to match ROS2 message and service definition requirements. This resulted in an update to the field names in the Port Drayage Plugin package.
o The Route Generator Worker class had a mix of Snake case and Camel case function names. This PR has updated all functions to camelCase to match the style used in other Carma-platform classes.
- Issue 1819: Fixed the Crosstrack distances in world model Trackpos object which returned wrong sign when logic in the method assumes a right handed frame.
- Issue 1846: Fixed Motion computation's parameters that are not set in worker class during runtime updates
- Issue 1847: Fixed the Roadway-objects node and Traffic-Incident-parser crash on receipt of semantic map in Cabin release branch.
- Issue 1851: Fixed the platooning gap calculation which does not consider vehicles in between and gap calculation is always assuming the preceding vehicle is the leader.

Carma-Cloud:

Enhancements in this release:

- Issue 31: Implemented IHP2 Speed Harmonization algorithm in which Carma-cloud application listens to incoming traffic control requests (TCRs) from vehicles, and responds with traffic control messages (TCMs) that has the calculated advisory speed using speed harmonization algorithm.
Fixes in this release:
- Issue 27: Fixed Occasional large delay experienced between CARMA Cloud receiving a TCR from V2XHub and CARMA Cloud sending all corresponding TCMs to V2XHub.

Autoware.ai:

Enhancements in this release:

- Issue 221: Added ROS2 support for Autoware build flags.
- Issue 222: Added Carma Utils Timer interfaces into ROS2 so that they can be used in Carma wm ctrl and Localization manager.

Fixes in this release:

- Issue 27: Fixed intermittent large delay experienced between CARMA Cloud receiving a TCR from V2XHub and CARMA Cloud sending all corresponding TCMs to V2XHub.



Version 4.1.0, released June 1st, 2022
----------------------------------------

**Summary:**
CARMA Platform release version 4.1.0 adds Personal Safety Message (PSM) support to CARMA Platform and CARMA Messenger along with updates to several CARMA Plugins to support ROS2 compatibility. Along with the aforementioned enhancements, several bug fixes and CI related enhancements are included in this release.

CARMA Platform:

Enhancements in this release:
- Issue 1757: Updated port of Roadway objects node to ROS2 to support ROS2 migration.
- Issue 1754: Updated basic autonomy library (and its ROS1 dependencies) to support ROS2 implementation.
- Issue 1762: Updated LCI (Light Controlled Intersection) Strategic Plugin so that it uses the schedule message from the intersection for setting its ET algorithm parameters instead of computing them based on SPAT if the message is available.
-	Issue 1672: Added J2735 Personal Safety Message (PSM) support to CARMA Platform and CARMA Messenger such that PSMs can be received and converted to an external object.
-	Issue 1697: Added Re-Routing Functionality to Route Following Plugin such that plugin will generate a lane change or sequence of lane change maneuvers to get the vehicle back onto the shortest route path.
-	Issue 1669: Updated GNSS to Map Convertor node from ROS1 to ROS2. 

Fixes in this release:
-	Issue 1771: Fixed vector map after switching ros1_bridge to dynamic bridge which will not make it to the ROS2 nodes which disables the object detection system.
-	Issue 1765: Fixed Rviz which continues to display the object marker even after the object is no longer being published. 
-	Issue 1760: Fixed few errors in motion computation node as well as BSM generator so that the accurate external object can be created using BSM messages.
-	Issue 1725: Fixed the ET (Entering Time) in (Light Controlled Intersection) Strategic Plugin.
-	Issue 1696: Fixed the planning for a lane follow maneuver starts close enough to the end of the maneuver such that there are only one remaining point to be added in path.
CARMA Messenger:

Enhancements in this release:
-	Issue 142: Added PSM parameters in carma_messenger_core and a debug log statement for recording timestamp of incoming Personal Safety Message (PSM).
   Fixes in this release:
-	Issue 138: Fixed PSM Parser and Elevation default values, also a check is added to make sure positional accuracy fields are not checked if the values don’t exist in the incoming message.

Autoware.ai:

  Fixes in this release:
-	Issue 220: Fixed the Georeference Topic that has to be published as a timer (every 2 seconds) so that the ROS Bridge can successfully publish the message.


Version 4.0.3, released May 10th, 2022
----------------------------------------

**Summary:**
Carma-platform release version 4.0.0 is first version that starts of the transition of system to ROS2 with V2X, Object Perception and some driver nodes transitioned to ROS2 and others still using ROS1 with communication enabled using ROS bridge. This release includes feature enhancements in support of the following proof-of-concept applications to demonstrate the following TSMO use cases:
- Cooperative Traffic Signaling (CTS), fixed signal traversal.
- Commercial Motor Vehicle (CMV) - Work Zone
Along with the above enhancements, several bug fixes are included in this release.
Note: V2X Hub release 7.2 includes CARMA streets plugin for following operations:
- Enhancement to receive, decode and forward the Traffic Control Message (TCM) and Mobility Operations Message (MOM) to enable lane restrictions by vehicle type and to record and notify the vehicle acknowledgement of receiving a TCM from infrastructure.

Carma Platform:

Enhancements in this release:

ROS2 - V2X, Object Perception, Drivers: Developed a ROS2 version of CARMA Platform capable of running on the Lexus RX450h with AutonomouStuff Pacmod3. Specifically, this milestone focused on the creation of a hybrid ROS1 and ROS2 system where the V2X, Object Perception, and core drivers are all ported to ROS2. This milestone included the following enhancements and defect fixes:

- Issue 1687: Updated the SSC Wrapper to support the ROS2 versions of SSC and Lexus Pacmod while preserving the ROS1 version as well.
- Issue 1500: Integrated driver_discovery and health_monitor behavior into subsystem_controllers/driver_controller
- Issue 1645: Launch ROS2 novatel GPS driver
- Issue 1580: ROS2 V2X Stack migration to ROS2
- Issue 1557: ROS2 Object Perception Stack migration to ROS2
- Issue 1277: CARMAWeb UI - Logout should issue a shutdown of the platform
- Issue 1689: Twist Filter node ms to mph speed limit
- Issue 1701: UI based remote launch (non-debug) fails to launch containers
- Issue 1703: UI appears to duplicate /system_alert notifications

Cooperative Traffic Signaling (CTS), fixed signal traversal: Upon receiving Signal Phase and Timing (SPaT) information (fixed plan information), a vehicle plans a maneuver to proceed through the intersection as efficiently as possible, or come to a safe stop if needed. This milestone include the following enhancements:

- Issue 1587: Signalized Intersection support in world model to handle SAE J2735 MAP and SPaT messages.
- Issue 1528 Feature/signalized intersection – Signalized Intersection regulatory element has been implemented to support the signalized intersection.

CMV Work Zone – Enhanced features to demonstrate CMV’s interaction with work zones to reduce speeds, adjust its trajectory to change lanes and adhere to lane use restrictions. The CMV will also demonstrate the capability to acknowledge receipt of work zone geofence information which will include the reduced speed and lane use restriction information.

- PR 1636: Update WMBroadcaster to process received TCM with a restricted lane
- PR 1682 & 1691: CMV broadcast acknowledgement after processing the incoming TCM

Fixes in this release:

- Issue 1608: The Web UI does not notify the user when the vehicle has left the route
- Issue 1634: Route is not selectable due to vehicle position not being set.
- Issue 1650: When CLC follows ILC in a trajectory plan, the first CLC TrajectoryPlanPoint has an abnormally high speed

Carma-Cloud:

Enhancement in this release:

- PR 12: CARMA Cloud sets up Lane Use Restriction to add, remove and update a lane use restriction for vehicles in a work zone area. Enhancement also has been made to receive TCM acknowledgement via the REST interface and log it in the logs.


Version 3.11.0, released Feb 3rd, 2022
----------------------------------------

**Summary:**
Carma-platform release version 3.11.0 is the final version which is exclusively ROS1. It has one major enhancement which is to enable Cooperative Right-of-Way (CRW), stop sign intersection traversal.This release also includes new enhancements in Carma-streets, as listed below, to aid with the stop sign intersection traversal.Subsequent versions will contain both ROS1 and ROS2 code as the system is transitioned from the former to the latter. In addition to the stop sign intersection traversal enhancements, several bug fixes are included in this release.

Note: V2X Hub release 7.1.0 includes a CARMA streets plugin for the following operations:
-	Receive, decode and forward the BSM, Mobility Operations Message and Mobility Path Message to CARMA Streets.
-	Broadcast schedule plan using Mobility Operations Message received from CARMA Streets.

CARMA-Platform:

Enhancements in this release:
-	Issue 1563: The following new Plugins and updates have been added to the CARMA code bases:
1.Added Stop Controlled Intersection Strategic Plugin to communicate with CARMA Streets that includes broadcasting the status and intent of the vehicle. 
-	Receiving schedule messages from CARMA Streets and processing them.
-	Generate maneuvers based on the received schedule for approaching the intersection and stopping at the stop bar through the Strategic Plugin.
2.Added Stop Controlled Intersection Tactical Plugin in CARMA Platform for generating trajectories according to the Trajectory Smoothing (TS) logic.
-	Issue 1584: Updated stop and wait plugin with a moving average filter to smooth the stopping behavior.

Fixes in this release:
-	Issue 1519: Fixed both Mobility Path and Mobility Operation header host BSM Id by changing the length from 10 digits to 8 digits with total length to send cpp message node. 
-	Issue 1552: Fixed Mobility Path encoder error by Llimiting the number of mobility path offset messages to 60.
-	Issue 1569: Fixed the BSM speed issues by updating the BSM generator launch files. 
-	Issue 1572& 1573: Updated parameters and logic to stop and wait plugin to prevent acceleration when the vehicle was trying to slow down.
-	Issue 1582: Fixed vehicle stops before 3 meters away from the stop line and wait for intersection access. 
-	Issue 1592: Fixed Stopping behavior of vehicle at intersection by updating the parameters of involved plugins to minimize jerkiness and also ensure the vehicle stops smoothly.

CARMA-Streets:

Enhancement in this release:
-	Issue 86: Added an open source software to monitor Kafka traffic to collect performance data and calculate metrics.Also added environment variables to set Kafka log retention time. 
-	Issue 87: Added a message logger service to the scheduling service to log scheduling logic calculations through a CSV log file for every scheduling calculation.

Fixes in this release:
-	Issue 72:  Fixed Sonar scan Analysis on the intersection model source code by excluding Open API generated code for the REST server under the intersection model directory.
- Issue 69: Fixed distance to end of Lanelet calculation since it sometimes generated the incorrect value, also removed code that is not needed.
-	Issue 79: Fixed delayed mobility path messages since it sometimes arrives more than 0.1s later than the mobility operation message which affects the mapping function between these two messages.
-	Issue 83&84: Updated frequency parameter in manifest JSON file to configure frequency of sending scheduling plans and set the scheduling delta to 0.2 sec.
-	Issue 88: Removed two Kafka topics v2xhub_in and v2xhub_out initially created to demo the CARMA-Streets V2X-Hub plugin’s capability to transmit J2735 messages to a CARMA-Streets deployment via Kafka.
-	Issue 92: Fixed Min 300 to max 900 milliseconds delay in these lanelet2 related functions for one vehicle testing. When one or more vehicles sends a message concurrently, the delay can be incrementally larger. 


Version 3.10.0, released Dec 17th, 2021
----------------------------------------

**Summary:**
Carma-platform release version 3.10.0 is comprised of two major enhancements. First, ROS1 Noetic (Updating the underlying ROS version from ROS Kinetic to ROS Noetic). Second updating the underlying OS from Ubuntu 16.04 to Ubuntu 20.04 to support the ROS2 migration which will use Ubuntu 20.04. Along with the above enhancements, several bug fixes and CI related enhancements are included in this release.

Enhancements in this release:
-	The following changes have been made to the CARMA code bases. 
1.	Docker base images updated to Ubuntu 20.04 and CUDA 11
2.	ROS version changed to ROS Noetic
3.	All python code updated from Python 2 to Python 3
4.	All ROS package manifest files updated from version 2 to 3
5.	Autoware.ai version updated from 1.13 to 1.14 except for the critical NDT node which remains on version 1.13 due to instability in the 1.14 version encountered during testing.
6.	Lanelet2 HD maps library updated from v0.9 to v1.1.1
7.	AVT Vimba Camera driver SDK version updated from v3.1 to v5.0
8.	Default C++ version increased from C++ 11 to C++ 14
9.	Handful of small code smells and compile warnings resolved.
10.	Default ROS workspace structure changed so autoware.ai and carma-platform exist in the same source directory

Fixes in this release:
-	Issue 1496: Fixed No objects reported from lidar detection anomaly
-	Issue 1490: Fixed platooning control package is missing autoware_msgs dependency
-	Issue 1486: Fixed localization in release/elise is less stable then in 3.8
-	Issue 1485: Fixed avt_vimba_camera driver is highly unstable in release/elise anomaly
-	Issue 1484: Fixed Cuda version mismatch in Elise
-	Issue 1477: Fixed Noetic version of carma-ssc-interface-wrapper-driver does not contain pacmod3 and kvaser_interface
-	Issue 1436: Fixed Mock Controller Driver Dropping Connection Due to Message md5sum Mismatch.

Version 3.9.0, released Dec 5th, 2021
----------------------------------------

**Summary:**
Carma-platform release version 3.9.0 is comprised of one major enhancements. First, Updated Carma Freight Port Drayage plugin web service integration. Along with the above enhancement, several bug fixes and CI related enhancements are included in this release.

Enhancements in this release:
-	Issue 1438: Updated Port Drayage Plugin to publish UI Instructions to the Web UI when a route to a newly received destination has been generated.
-	Issue 1416: Update Port Drayage plugin to receive PICKUP actions and to broadcast an arrival message when arriving at a 'PICKUP' location.
-	Issue 1321: Added capability for CARMA Platform to generate a new active route based off a received destination point for Port Drayage.
-	Issue 1252: Added incoming Mobility Operation message processing to Port Drayage Plugin when new Mobility Operation message with strategy carma/port drayage received by a carma vehicle and contents of the Mobility Operation message's strategy params field are stored in Port Drayage Worker.

Fixes in this release:
-	Issue 1534: Updated Port Drayage plugin to set its cmv_id using the vehicle_id global parameter, which includes the vehicle's license plate information and updated cmv_id to a string value.
-	Issue 1479: Added combined_lidar_frame arg name in the launch files to have default frame for passenger vehicles and freightliners.
-	Issue 1520&1515&1521:  Fixed a small bug in CLC “duration is out of 32-bit range" (from vehicle state having a speed of -nan in ILC logs) ~1-2 seconds after starting the lane change.
-	Issue 1507: Fixed Input vectors empty exception sometimes at the end of the lane change from cooperative lane change plugin.
-	Issue 1506: Fixed Insufficient spline points exception from inlane-cruising when executing a lane change.
-	Issue 1524: Fixed route node’s logic using non-down track calculation for route completion check. Which is using a different down track frame compliant calculation for its end of route check.
-	Issue 125: Updated route event popup text from Restart to "Start A New Route" in web UI.
-	Issue 91: Fixed strategy params parsing issues in carma-platform with replacing encoded quotation marks in received Mobility Operation message's strategy params field.


Version 3.8.2, released Oct 22nd, 2021
----------------------------------------

**Summary:** 
Carma-platform release version 3.8.2 is a hotfix release for 3.8.0.

Fixes in this release:
-	Issue 1489: Fixed stop and wait tactical plugin which uses constant vehicle acceleration specified through config parameters to bring the vehicle to a stop by sending acceleration value to the tactical plugin as a meta data in the maneuver message.
-	Issue 147: Updated the IP Addresses listed in the drivers. Launch file for the forward-facing left and right cameras on two of the trucks (10004 and 80550).
-	Issue 75: Fixed two LiDAR’s which were not calibrated on the carma white truck during verification testing.


Version 3.8.1, released Oct 15th, 2021
----------------------------------------

**Summary:** 
Carma-platform release version 3.8.1 is a hotfix release for 3.8.0.

Fixes in this release:
- Issue 1461: Fixed basic autonomy errors in inlane cruising to prevent planning maneuvers on top of themselves, by fixing broken maneuvers_to_points method that didn't properly track the processed lanelets.
- Issue 1446: Fixed the work zone plugin by updating vehicle length parameter and modifying traffic light downtrack calculations to ensure that the target stopping location would be shifted backward by vehicle length.
- Issue 1475 & 139: Fixed carma-platform and carma-config by carma-docker.launch file for each freightliner to pass in the correct LiDAR frame and fixed IP address of the front left camera in the blue truck.

Version 3.8.0, released Sep 24th, 2021
----------------------------------------

**Summary:**
Carma-platform release version 3.8.0 is comprised of two major enhancements. First, Cooperative Traffic Signaling (CTS), fixed signal transit for Work Zones using a SPaT message a vehicle plans a maneuver to proceed through the intersection as efficiently as possible, or come to a safe stop if needed. Second lane geometry updates affected by a geofence to split and stitch lanelets together to match the geofence requirements. Along with the above enhancements, several bug fixes and CI related enhancements are included in this release.

Enhancements in this release:
-	Issue 1366: Added Work Zone use case node that can process intersection transit maneuvering messages sent from Carma Cloud.
-	Issue 1363: Updated Work zone use case related map modification in Geofence, Developed logic to identify which lanelets are affected by a Geofence which involves a geometry change and how to split stitch lanelets together to match the work zone Geofence requirements.
-	Issue 1377: Updated Strategic plugin to use Plan Maneuvers service call to include the initial vehicle state at the start of planning and removed duplicated code to get the current position/lane/velocity information in order to begin maneuver planning.

Fixes in this release:
-	Issue 1172: Updated Stop and Wait plugin to use a constant acceleration for planning stopping maneuvers at the end of a route and also updated In lane cruising to prevent overlapping points on lanelet transitions
-	Issue 1371: Fixed both carma-platform and carma-msg’s Maneuver Parameters message value “negotiation type” is incorrectly spelled as "neogition_type" in multiple files.
-	Issue 1383: Fixed Ci failing due to package of pure pursuit jerk wrapper in SonarQube.
-	Issue 1461: Fixed basic autonomy introduces errors in in lane-cruising
-	Issue 1420: Fixed Platform and UI need to process/display the SPAT correctly and distinguish which SPAT is from the approaching traffic signal controller for the SV as traffic signal intersection ID and group ID that are not available.

Version 3.7.2, released Sep 1st, 2021
----------------------------------------

**Summary:** 
Carma-platform release version 3.7.2 is a hotfix release for 3.7.0.

Fixes in this release:
-	Issue 1426: Route Following Plugin can seg fault in the presence of a lane change after a reroute.
-	Issue 1427: Rerouting triggered multiple time from TIM geofence.
-	Issue 1428: Excessive steering during lane change along curve at ACM 

Version 3.7.0, released Aug 10th, 2021
----------------------------------------

**Summary:** 
Carma-platform release version 3.7.0 is comprised of three major enhancements. First, Unobstructed lane change. Second Cooperative Lane Follow (CLF) - All Predecessor Following (APF) platooning. Third, Cooperative Traffic Management - Speed Advisory.  Along with the above enhancements, several bug fixes and CI related enhancements are included in this release.

Enhancements in this release:
-	Issue 1344: Added Platooning Strategic and developed Tactical and Control Plugins to enable CARMA Platform to perform platooning.

Fixes in this release:
-	Issue 957: Fixed Platooning code missing some of the mobility message fields where Platooning plugin messages are populated with correct data.
-	Issue 1347: Fixed route following plugin and Inlane cruising plugin with error no points to fit in time span exception during platooning test.
-	Issue 1351: Fixed Route Following Before passing the maneuver to the response the starting speed is updated but the ending speed is not changed and doesn't update speed limit if route not updated.
-	Issue 1356: Fixed Routing Graph creation takes unacceptably long time on ACM map.
-	Issue 1360: Fixed Smooth speed change when the control plugin first changes to platooning control.
-	Issue 1364: Fixed Platoon gap control by adding last speed command as current speed for smooth speed transition.
-	Issue 1227: Fixed cooperative lane change functionality by adding necessary components relates to yield plugin and fixed few other issues for CLC and integration tested.

Version 3.6.0, released June 29th, 2021
-------------------------------------

**Summary:**
Carma-platform release version 3.6.0 is comprised of four major enhancements. First, Added ADS unobstructed lane change. Second CTM Move-over law –When receiving a request from an emergency vehicle, CARMA Platform plans move over to the adjacent open lane. Third, Added Geofence speed, Gap control and lane closure. And fourth, added Carma-cloud integration. Along with the above enhancements, several bug fixes and CI related enhancements are included in this release.

Enhancements in this release:
-	Issue 1195: Added new functions to World Model interface, like route conversion to map and sample Route Points.
-	Issue 1199: Added a new node that visualizes host's and incoming mobility path’s location and received mobility path is synchronized to that of the host by matching the time steps and interpolating the points.
-	Issue 1206: Added a debug topic to in-lane cruising to improve the data analysis experience. 
-	Issue 1209: Updated Yield plugin to receive adjustable inter-vehicle gap from the map and modify the trajectory accordingly.
-	Issue 1216: Added the new Carma node handle spin behavior which improved vehicle control by reducing planning and feedback communications latencies. 
-	Issue 1234: Added lane change status publisher to Yield plugin constructor to ensure the topic is published properly.
-	Issue 1235: Added a ROS parameter for choosing the tactical plugin to be used for lane changing.
-	Issue 1275: Updated WM Broadcaster logic to determine when the host vehicle is within an active Geofence.
-	Issue 1296: Updated TCM Path nodes to match the logic of Carma-cloud with absolute Cartesian coordinates for each nodes.

Fixes in this release:
-	Issue 1163: Modified the object detection tracking node to accurately relay the object id for classification.
-	Issue 509: Arbitrator doesn't handle shutdown state properly.
-	Issue 1217: The EKF node and the pose to tf node both appear to be outputting the same transform which results in a frequency of 100Hz which will likely have a negative impact on system performance due to the extra high data frequency.
-	Issue 1223: Fixed traffic incident parser node georeference remap to /map param loader/georeference.
-	Issue 1231&866: Control requests are supposed to be published at periodic intervals (10s) after a route is selected.
-	Issue 1232: Fixed a number of issues with the TIM use case functionality. Correctly sets the schedule for the control messages coming out of the traffic incident parser node.
-	Issue 1244: Motion prediction reports wrong speed on prediction state part of roadway object message.
-	Issue 1267: Fixed Incorrect node placement from traffic incident parser node.
-	Issue 1270: TIM scenario vehicle speed up to configured limit instead of slowing down when in Geofence region.
-	Issue 1283: Fixed TCR timer after selecting a route, the WMB should broadcast constantly every 10 seconds.

Version 3.5.3, released April 9th, 2021
----------------------------------------------

**Summary:**
carma-platform release version 3.5.3 is comprised of five major enhancements. First, Added Automated Driving System (ADS) Lane Follow. Second Added ADS Motion Control. Third, ADS Perception. Fourth, ADS Planning. And fifth Operator UI. Along with the above enhancements, several bug fixes and CI related enhancements are included in this release.

Enhancements in this release:
- Issue 731: Updated the platooning tactical plugin to be compatible with the new routing mechanism of CARMA that uses vector map instead of recorded waypoints.
-	Issue 1022: Added Tactical Stop and Wait Maneuver Plugin that handles maneuver plans of type Stop and Wait.
-	Issue 1023: Added filter to the incoming trajectory plan request so inlane cruising only works on lane following maneuver types.
-	Issue 1031: Added a new carma_record package with rosbag record functionality which allows for carma config parameters to control the recording of certain Topics.
-	Issue 1037&1038: Added Traffic Incident parser node is a standalone ROS node in the CARMA Platform which collects incoming broadcasted data from DSRC Driver also added Speed advisory.
-	Issue1042: Added waypoint generation methodology to unobstructed lane change and updated the Lane Change Tactical Plugin to behave in a compatible manner with the new Guidance waypoint generation methodology.
-	Issue 834&1049: Updated the in-lane cruising plugin to include object avoidance logic.
-	Issue 1056: Added Feature mobility conversion to support the cooperative lane change design with Mobility Path data to get accurate predictions of vehicle motion for external object prediction.
-	Issue 1089: Developed Yield tactical plugin for modifying trajectories to avoid surrounding objects and Updated In-Lane Cruising plugin to communicate with Yield plugin.
-	Issue 1168: Added lane change status to the cooperative lane change plugin for displaying progress in the UI.
-	Issue 1140: Added new GPS only with initialization mode this allows the GPS offset with the map to be computed which gives the resulting GPS only pose far more accuracy. 
-	Issue 1072: Added Additional logic to include the camera as a required driver in the health monitor node and system recognizes the camera's driver status sends the appropriate alert messages to the health monitor.

Fixes in this release:
-	Issue 992: After receiving a response from the truck, the appropriate truck image does not always load and a broken link is shown.
-	Issue 1057: After getting a left route event unable to reselect the same route on the UI as receiving the already following route error.
-	Issue 1060: where the vehicle doesn't publish a route completed event after stopping at the end of the route.
-	Issue 1062: Non-valid route files reported from route node to UI as "0".
-	Issue 1028: On implementing the end of route behavior Trajectory executor issues error "Ran out of trajectory" at route end.
-	Issue 1189: Restrict routes which consist of a shortest path with duplicate lanelet IDs from being generated
-	Issue 1114: In-Lane cruising does not always select the shortest path lanelet when there is an adjacent lanelet going in the same direction.
-	Issue 1130: Fixed the required plugins configuration file to fix the UI display of required plugins to have Route Following, InLane Cruising Plugin, Stop and Wait Plugin, Pure Pursuit.
-	Issue 1160&1164: The predicted and current velocities of the roadway obstacle when detected on the /environment/roadway objects topic is always zero.

Version 3.4.2, released December 15th, 2020
--------------------------------------------------------

**Summary:**
carma-platform rlease version 3.4.2 is a hotfix to update platform with the most recent version of carma-messenger (carma-messenger-1.0.2). The carma-messenger-1.0.2 release resolves a defect where version numbers were not populating in carma-messenger-ui.

Fixes in this release:
-   Issue 948: The carma user interface dispalys "NULL" instead of the current version number.

Version 3.4.1, released December 11th, 2020
--------------------------------------------------------

**Summary:**
carma-platform release version 3.4.1 is a hotfix release to address the issue of unexpected switching between NDT and GPS localization. This hotfix ensures that switching between NDT and GPS will only occur in the event of a timeout.

Fixes in this release:
-   Issue 1005: The gnss_ndt_selector would cause lidar to gps failover in the event of an ndt timeout and vice versa regardless of the    operational mode (NDT only, GNSS only, AUTO score based switch).

Version 3.4.0, released December 9th, 2020
--------------------------------------------------------

**Summary:**
carma-platform release version 3.4.0 is comprised of four major enhancements. First, a steering limiter was added in order to prevent Carma from taking turns that present a risk of vehicle rollover. Second, updates were made to the health_monitor to allow for graceful handling of sensor failure, including control handover to the human driver. Third, carma-messenger was updated to detect nearby automated vehicles via DSRC. And fourth, functionality was added to carma-messenger to allow for requesting, receiving, and displaying truck safety info from a passing Carma truck via DSRC while vehicles are in motion. Along with the above enhancements, several bug fixes and CI related enhancements are included in this release.

Enhancements in this release:
-	Issues 520 & 617: Added a steering limiter to prevent Carma from taking overly sharp turns, reducing rollover risk.
-	Issue 592: Updated Carma’s UI to work with the new Route Manager.
-	Issue 606: Created carma-messenger repo and docker images to separate v2x messaging stack from carma-platform.
-	Issues 613 & 688: Implemented Truck Inspection Client for carma-messenger.
-	Issue 625: Added a LightBar Manager to handle control of the lightbar between different Carma components.
-	Issue 640: Updated the health_monitor to better handle lidar failures.
-	Issue 766: Improved localization computation speed.
-	Issue 838: Added support for PCD map file generation.
-   Issue 986: Added ability for gnss_ndt_selector to operate with a degraded sensor.

Fixes in this release:
-	Issue 460: carma-platform does not fully shutdown when a fatal system alert occurs.
-	Issue 588: Automatic initialization/localization is inaccurate, and manual localization is taking too long.
-	Issue 668: System_alert does not detect car or truck configuration.
-	Issue 758: The platooning strategic plugin cannot be built with docker build.
-	Issue 835: The default rviz configuration uses the wrong initialpose topic.
-	Issues 871 & 877: The MobilityOperation and MobilityRequest messages generated by the truck inspection client cannot be encoded or decoded by the message node.
- Issue 908: The health monitor continuously publishes alerts until an alert status changes.
- Issue 1002: Rosbridge crashes at startup due to vtk package update.


Version 3.3.0, released February 7th, 2020
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
- Issue 499: Trailer angle sensor devices are not mapped on the host or in docker
- Issue 506: The carma_build script is broken on autoware.ai fork v1.13
- Issue 514: Health Monitor param mismatch
- Issue 517: Load vector map to be loaded when carma is launched


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
- PR 330: Remove utility packages and update checkout.bash to depend on CARMAUtils
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
- PR 15: Update checkout.bash script and CARMAUtils repo name
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
- PR 21: Updating checkout.bash script and CARMAUtils repo name
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
