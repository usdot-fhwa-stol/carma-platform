CARMA System Release Notes
----------------------------

Version 4.5.0, released April 10th, 2024
----------------------------------------

### **Summary**

This release represents a significant advancement in utilizing cooperative perception and cooperative driving automation (CDA) to enhance the safety of Vulnerable Road Users (VRU) at signalized intersections, as demonstrated with CDASim. Key features include the implementation of cooperative perception using both infrastructure and vehicle sensors, along with data fusion (DF) and the encoding and decoding of sensor data sharing messages (SDSM). These advancements facilitate the effective sharing of VRU state information, collected by infrastructure sensors, with nearby connected road users, particularly those in CDA-equipped vehicles. This establishes a state of Cooperative Perception (CP). The primary goal is to improve overall road safety, especially by reducing the risk of collisions between vehicles and VRUs such as pedestrians and cyclists. It is important to note that the functionalities developed in this release were only tested in a simulation environment, and not all of them are currently directly portable to a real-life environment.

### **CDA Sim** 

This release introduces new functionalities of CDASim, including the ability to spawn sensors in CARLA and transmit detection data to vehicle and infrastructure actors. It also introduces a new functionality of the CARLA Scenario Runner, allowing for the configuration of CARLA scenarios and the collection of scenario metrics.

Enhancements in this release: 

- Issue 211: CARLA Sensor Integration: This enhancement adds functionality to CDASim to create and poll detection data from sensors deployed in CARLA. Currently, Lidar is the only supported sensor type integrated with CARLA. The update includes the development of CARLA Ambassador functionality, which requests the creation of sensors based on received SensorRegistration Interactions and publishes Detection Interactions for each detection from the created sensor.
- PR 164: Updated logback settings to include fully qualified class name and line number to log statements.
- PR 170: Added cfg that the carma-config is configured to look for at the moment. This is sumo intersection 916 fixed signal configuration.
-	PR 182: Enhanced behavior of the sumo traffic light for TRB scenario.
-	PR 183: Added new infrastructure ambassador config and logging.
-	PR 186: Updated CARMA Ambassador to forward time sync messages to CARMA Platform.
-	PR 191: Updated docker build to allow for caching of installed dependencies to improve the time it takes to rebuild CDASim.
-	PR 193: Removed EVC Sumo from the CDASim repository and moved to its own repository https://github.com/usdot-fhwa-stol/evc-sumo.
-	 PR 205: Added time sync logs for VRU data analysis.

Fixes in this release:

-	PR 151: The current CDASim can synchronize vehicle positions and orientations between Carla and SUMO simulations. Fixed the issue with the correct synchronization of SUMO IDs and Carla role names. 
-	PR 154: Added configuration files for sumo GUI settings to avoid.
-	Issue 540: Set delay to 1000 ms per second to startup SUMO running at approximately real-time to avoid time sync issues with simulation and infrastructure
-	PR 171: Update GitHub action workflows to avoid duplicate or unnecessary actions.

Known Issues related to this release: 

-	CARMA Platform Issue 2117: Data analysis revealed that commanded acceleration exceeded anticipated value range.
-	CARMA Platform Issue 2118: Vehicle command frequency varies unexpectedly from target 30Hz.
-	CARMA Platform Issue 2119: CARLA initialization causes random time offset from CDASim provided simulation time.
-	CARMA Platform Issue 2323: Yield plugin can sometimes detect vehicle is passed but can detect collision again.
-	CARMA Platform Issue 2345: Yield is not decelerating at comfortable rate even with CP enabled.
-	CARMA Streets Issue 409: Inaccurate minimum end time in SPaT received from EVC.
-	CARLA Sensor Lib Issue 16: Sensorlib is dropping some detections sometimes once per 3-5 second window.
-	Multiple Object Tracking Issue 145: Angle values do not wrap properly. Revisited orientation representation. When averaging or otherwise working with angular values, the library does not properly handle wrapped values.

### **CARMA-CARLA Integration Tool** 

This release introduces CARLA Sensor Integration on object-level data using the carla-sensor-lib library (new repository in this release as well) for CARMA vehicles. It also updates outdated packages, adapts to the ROS2 naming convention in line with the CARMA Platform's updates, and improves the stability of the tool. This enhancement leverages scripts to wait for CARLA, eliminating the need for fine-tuned sleep commands.

Enhancement in this release: 

-	Issue 38 / PR 40: Renamed plugins to accommodate ROS2 naming convention in carma-platform
-	Issue 39 / PR 40: Added a configurable start delay for the carma-platform's system to be fully ready - route is selected and semantic map is available in all plugins/nodes. Without it, the carma-platform shuts down automatically almost half of the time.
-	PR 41: Added detection node that leverages carla-sensor-lib to publish object level detection data, replacing the current external objects node
-	PR 44: Ackermann controller was throwing a KeyError when carma-base image version was changed to develop from 4.2.0. To fix this issue, the install.sh was modified to include specific version 1.0.1 of "simple-pid" package and the carma_carla_agent.launch was updated to include the newly added start delay parameter
-	PR 58: Adjusted config to use map centric instead of sensor centric object detection
-	PR 67: Added script to wait for CARLA. This script can replace sleep commands that were environment specific
-	PR 68: Added workflow dispatch as carma-carla-integration tool depends on libraries in carla-sensor-lib. It should trigger rebuild if carla-sensorl-lib repo has new changes pushed.

Fixes in this release:
-	PR 64: Fixed error log statements caused by incorrect syntax in launch file.

### **CARMA NS3 Adapter** 

Enhancement in the release:
-	PR 11: Added udp listener to listen to time sync messages from CDASim and broadcast it in rostopic as /sim_clock.
-	PR 13: Added wave JSON configurations for SDSM.

### **CARLA Sensor Library** 

The carla-sensor-lib is a new repository introduced in this release, housing a wrapper library for CARLA sensors along with a Docker image. This library provides functions for creating CARLA sensors with noise modeling and retrieving object-level detection information from these sensors. The CARLA CARMA Integration tool utilizes this library to feed object-level data to the CARMA Platform. Additionally, this repository generates a Docker image that deploys an XML RPC server, enabling clients to create sensors in CARLA and poll their detections. This feature is designed to facilitate object-level detection for CDASim deployment, making the data available to other integrated simulators or software systems under test.

### **CARLA ScenarioRunner** 

The scenario-runner is a new repository and Docker image introduced in this release. This directory contains custom ScenarioRunner scenario configurations designed to facilitate integration testing interactions between CARMA vehicles and Vulnerable Road Users (VRU) in a signalized intersection. These scenarios primarily serve as example references. The code for ScenarioRunner is housed in the srunner Python package, with the scenario_runner.py script responsible for launching the scenarios.

### **EVC-SUMO (New Private Repository)** 

EVC-sumo is a new private repository featuring the Econolite Virtual Controller (EVC)-SUMO bridge tool, part of the XiL co-simulation toolset. This bridge acts as a mediator between EVC, accessed via the PyEOS Python package, and SUMO, facilitating the integration of traffic light and detector status exchanges.

Currently, building and using the Docker image for this tool requires a proprietary GitHub token, accessible only to members of the USDOT-FHWA-STOL organization, due to licensing restrictions imposed by Econolite.

### **Multiple Object Tracking** 

The Multiple Object Tracking repository introduces a new library designed to support cooperative perception. This library is tailored for tracking multiple objects from various sources, utilizing a standard cooperative perception object tracking interface. Inputs can include object-level data from diverse sources such as the J2334 Sensor Data Sharing Message, Basic Safety Message, or local perception. This versatility allows the library to be deployed across different road actors, such as C-ADS equipped vehicles or infrastructure, using message adapters appropriate for the respective middleware.

The library features multiple submodules and functionalities that leverage architectural and algorithmic advancements from the sensor fusion community. An example of how this library can be implemented to execute a complete multiple object tracking pipeline is demonstrated in the CARMA Platform.

### **CARMA Builds** 

CARMA Builds is a new addition to the CARMA ecosystem, designed to facilitate Docker image build coordination among various transportation users. This component provides essential Docker images for building other projects within CARMA, streamlining the development process and enhancing collaboration across different transportation initiatives.

### **CARMA Streets** 

The main new functionality in CARMA Streets is its ability to generate J2334 Sensor Data Sharing Messages from object-level data detected by simulation sensors. Additionally, it has received further enhancements, including a new dependency on CARMA Builds, which allows for efficient pulling from common base images. There are also minor improvements in time synchronization, data collection, and JSON parsing, enhancing the overall functionality and efficiency of the system.

Enhancements in this release: 

-	PR 341: Added functionality to the TSC Service to periodically log vehicle and pedestrian call information in easily processable format for the Intersection Safety Challenge (ISC) data collection. Added a logger to log pedestrian and vehicle information in the following csv format. 
-	PR 352: Added new json utility library to CARMA Streets.
-	PR 353: Added streets_messages library which will house JSON serialization and deserialization functions as well as carma-streets messages. 
-	PR 361: Created base image for CARMA Streets services like Message Service, Intersection Model, and future Sensor Data Sharing Service that includes lanelet2 dependency
-	Issue 408: CARMA Streets SDSM functionality. Added Sensor Data Sharing Service to consume detection data and produce SDSMs.
-	PR 390: Added pip3 dependencies
-	PR 399: Added time sync topic to default list of topics for Kafka data collection script
-	PR 403: Added time sync logs
  
Fixes in this release: 

-	PR 332: Fixing docker builds for all XIL Release unused CARMA-Streets services.
-	PR 349: Updated build_scripts/install_dependencies.sh STOL APT functionality. STOL APT Debian packages including carma-clock-1 are not pushed to AWS bucket (repository) based on ubuntu distribution. Added functionality in script to get ubuntu distribution codename and use it for STOL APT repository path. See carma builds and actions repository for more information about s3 push workflows.
-	PR 397: Fixed Kafka data collection script

### **CARMA Platform** 

The main improvements for CARMA Platform in this release include a new package callled, carma_cooperative_perception, that can process both object list data from local perception and from J2334 SDSM, providing enhanced multiple object tracking functionality. Additionally, there have been further improvements to its world model and motion prediction stack, enabling it to recognize and respond to more objects than just vehicles (such as pedestrians), including movements not along the road (such as pedestrians moving on crosswalks). This feature has been tested at a signalized intersection in CARLA, incorporating traffic signal following behaviors using J2735 SPAT and MAP, demonstrating significant safety advancements in its operational capabilities.

Enhancements in this release: 

-	Issue 2007 / PR 2008: To properly integrate with the CARMA Platform’s communications stack, the CARMA Ambassador component responsible for receiving and transmitting messages through NS-3 on behalf of the CARMA Platform needs to know each vehicle’s ID within the CARLA environment. Added an initial “handshake” that must take place between the simulation comms driver and the simulation CARMA Ambassador. 
-	PR 2144: Update roadway_objects package structure
-	Issue 2151 / PR 2152: Added component tests for motion_computation package
-	PR 2242: Increased predicton period so that yield_plugin has enough data for VRU crossing pedestrian.
-	PR 2320: Added the visualization of fused objects and its motion prediction to rviz so the user can see it live.
-	Issue 2347: Add cooperative perception stack.
-	Issue 2153 / PR 2154: Improved yield_plugin functionality to consider objects' predicted_states, instead of assuming the roadway_object is moving along the route
-	PR 2313: Improve yield plugin performance to parallel process objects and optimize objects with cv prediction

Fixes in this release: 

-	PR 2091: Fixed sed regex in cloudscript, previously only filtered the first instance of each pattern matched
-	PR 2131: Removed the localization launch file and updates the ros2 launch file with the required simulation mode launch argument

Other Enhancements:

-	Continuation of porting ROS1 to ROS2:
     -	Issue 2090 / PR 2089: Ported the intersection_transit_maneuvering node from ROS1 to ROS2
     -	PR 2109: Ported vector and pcd map loader node launchers to ros2. The nodes themselves are defined in autoware.ai under the package map_file_ros2
     -	PR 2115: Ported dead reckoner to ROS2
     -	PR 2121: Ported random filter and voxel grid filter points downsampled to ROS2
     -	PR 2125: Ported ndt_matching to ROS2
     -	PR 2127: Ported ekf localizer to ROS2
     -	PR 2133: Enabled ROS2 rosbag logging. 
     -	PR 2134: Ported the health monitor's driver functionality to ROS2.
     -	PR 2173: Modified the CI build to skip the novatel_oem7_msgs and novatel_oem7_driver packages from the ros1 build.
-	PR 2160: Updated the cloud-sim deployment to allow multiple users to start cloud sessions for XiL.
-	PR 2132: Updated the install-docker.sh script (which is used as part of the process for setting up a new PC for CARMA and/or CARMA Messenger) to remove additional necessary docker packages
  
Fixes in the release:

-	PR 2350: UI is not able to select route due to system_alert publisher QoS type

### **CARMA Base** 

Fixes outside of this release:
-	PR 180: Installed four new packages within the carma-base, all of which are required for packages built as part of carma-platform:
       -	nmea-msgs and gps-tools are dependencies for the novatel_oem7_driver and novatel_oem7_msgs packages built in carma-platform.
       -	pybind11 and test-msgs are dependencies for building foxy-future rosbag2 overlay packages, which contain backported fixes that are not contained within the default foxy rosbag2 implementation.
-	PR 178: Added the velodyne pointcloud as a ros foxy dependency. The package is required for the lidar_localizer_ros2 package recently ported to ros2.
-	PR 184: Added the foxy mcap storage plugin required to store the recorded rosbag in mcap format.
### **CARMA Messenger**

Enhancements in this release:

-	Issue 205: Adds support for encoding/decoding J2334 SDSM and necessary message types.
Other Fixes:
-	PR 204: Added volume to the messenger ros2 container, allowing the container to access the vehicle calibration.
### **CARMA Msgs**

Enhancements in this release: 

-	Issue 205 / PR 208: Added new ROS message definitions for the SensorDataSharingMessage (SDSM) and its nested data frames and data elements according to the 2020 SAE J3224 specification.
-	Issue 215: Added new message types and rules to support the CARMA cooperative perception stack. 
-	PR 226: Added some convenience functions to convert the SAE confidence enumerated values to floating point equivalents
-	PR 221: Added a new ROS package with some Python functions to load ROS messages from YAML strings and files

Other Fixes:

-	PR 211: Added Github Actions CI workflows
-	PR 214: Fixed missing SKATEBOARD value type. Both SAE J2735 messages for HumanPropelledType were missing the "SKATEBOARD" value type which resulted in following types being off by a value of 1.
  
### **CARMA Torc Pinpoint Driver**

Other Fixes: 

•	PR 39: Fixed lat and lon truncation from being converted to a float. 

### **CARMA Time Library**

Other Fixes:

•	Issue 10 / PR 12: Fixed carma-clock object to support multiple threads waiting on initialization
### **CARMA Velodyne Lidar Driver**

Other Enhancements:

•	PR 110: The timestamp of the data from the sensor’s clock is used for point cloud ROS Message timestamps published by the velodyne driver.

Other Fixes

•	PR 106: Replace CircleCI with GitHub Actions workflow
•	PR 109: Removed installation of the velodyne_pointcloud package from this image. 

Version 4.4.3, released June 21st, 2023
----------------------------------------

### **Summary**
This release adds functionality for the integration of CARMA Streets and V2X Hub to CDASim. Notable features of this integration include time synchronization of CARMA Streets and V2X Hub with CDASim via a newly developed time synchronization library, full integration of the NS-3 network simulator to simulate the transmission of DSRC messages to and from CARMA Streets/V2X Hub and CARMA Platform, and integration of the Econolite Virtual Controller module to provide an NTCIP-compatible interface between CARMA Streets/V2X Hub and the SUMO traffic simulator.

### **CDA Simulation** 

The CDASim repository was previously called carma-simulation. This package name update has been applied to the GitHub repository and Docker Hub repository and images and will be the name for this package going forward.

This release for CDASim adds the necessary software components to register and communicate with one or more infrastructure software (V2X Hub and/or CARMA Streets) instances and fixes issues with the configuration and usage of the NS-3 simulator to simulate the transmission and reception of DSRC messages by simulation participants (both vehicle and infrastructure). This release also adds the necessary interfaces to support integration with the Econolite Virtual Controller, however the Econolite Virtual Controller itself is not distributed with this release. In addition, please note that this release of CDASim has been integration and verification tested with CARMA Platform 4.2.0, though the interfaces used should support all ROS2 versions of the CARMA Platform. 

Enhancements in this release: 
 - PR 108:  Added SUMO multi-client feature for EVC integration. 
 - PR 109: Added EVC-SUMO bridge. 
 - PR 110: Added GitHub actions workflows for CI/CD  
 - PR 114: Setup shell structure for infrastructure ambassador. currently this design mainly followed Carma ambassador implementation. 
 - PR 115: Implement Loop Detector Functionality in EVC-SUMO Bridge 
 - PR 120:  Updated install.sh script by removing unused code. 
 - PR 121: The implementation of Infrastructure Time Interface class which includes encoding time message and updating to registered instances.
 - PR 122:  Added functionality to enable the handshake between CARMA simulation and CARMA street. The data flow of the handshake is as follows:
    1.	CARMA Streets sends an infrastructure registration message to mosaic-infrastructure.
    2.	The infrastructure Registration Receiver receives the registration message from CARMA street and converts it to an infrastructure Registration Message object.
    3.	The Infrastructure Registration Receiver stores the infrastructure Registration Message object in a queue.
    4.	The Infrastructure Message Ambassador runs in a loop and checks the queue for any new messages while processing time advance grants.
    5.	When a new message is found in the queue, the Infrastructure Message Ambassador creates an RSU with its DSRC configuration and sends the registration message to the MOSAIC RTI.
    6.	These created RSUs are stored in the Infrastructure Instance Manager with the type of Infrastructure Instance.
 - PR 124: Added Build status checks in readme markdown page for CI and Docker hub.
 - PR 125: Added an NS3 docker build process and GitHub actions to build and run sonar analysis. Additionally, Upgraded SUMO version to 1.15.0, which now supports 'induction loop' for the purpose to test EVC-SUMO integration.
 - PR 126: Added new scenario files for EVC and fix a function call in bridge to get traffic light state from sumo.
 - PR 127: Refactored existing v2x-message reception logic into a new maven module to be easily incorporated into the infrastructure ambassador without code duplication.
 - PR 133: Added TraCI IP argument for EVC-Sumo connection.
 - PR 118: Updated Java version to 11 for maven sonar scanner plugin
   
Known Issues in this release: 

- CARMA Platform Issue #2117 Data analysis revealed that commanded acceleration exceeded anticipated value range.
- CARMA Platform Issue #2118 Vehicle command frequency varies unexpectedly from target 30Hz
- CARMA Platform Issue #2119 CARLA Initialization causes random time offset from CDASim provided simulation time.

Fixes in this release:

- PR 128: Fixed the simulation freezing issue that occurs when NS3 and infrastructure ambassador are enabled and running in the MOSAIC scenario Tiergarten which needs infrastructure ambassador to successfully register RSU and DSRC to NS3 and store the new registration in the infrastructure Instance Manager.
- PR 129: Updated infrastructure time message to use millisecond timestep instead of nanosecond time stamp since nanosecond level logic is not supported on infrastructure.
- PR 130: Resolved an error with the tokenizer in the CarmaV2xMessage class which causing it to incorrectly bundle tokens together and parse failure for valid message.
- PR 132: Fixed Parser error in mosaic Carma-utils to avoid parsing after reading the final field. This will ignore any potential junk data after the end of the payload field.
- PR: 135: Fixed a variety of issues related to NS-3 integration with CARMA Streets and platform. Integration is not functional to the point that messages from streets and platform can enter NS-3 and be simulated, and then exit NS-3 and be received by their respective systems.

### **CARMA NS3 Adapter** 

The carma-ns3-adapter is a new repository and docker image for this release, which came from an existing package that has been refactored out of the carma-platform repository to better match the structure used by other CARMA Platform drivers.

Enhancements in this release: 
- PR 1:  Implemented new feature to send the registration message each timestep by using two sockets in parallel.
- PR 3: Added configuration parameters in NS3 adapter for host IP address in registration message.
- PR 5: Updated NS3 adapter mode to load the parameters at launch and changed the IP address values for XIL testing.
- PR 4: Added GitHub actions workflows and configured Docker Hub repositories for NS3 adapter. 

### **CARMA Streets** 

CARMA Streets Traffic Signal Control (TSC) service has been integrated with CDASim in this release. This includes the necessary input data flows (from MOSAIC and from the Econolite Virtual Controller) as well as time synchronization and output data flows in the form of V2X messages.

Enhancements in this release: 

- Issue 331: Added new functionality for CARMA-Streets to consume Multi-Modal Intelligent Traffic Signal System (MMITSS) phase control message.
  
Fixes in this release: 

- PR 328:  Fixed time sync segfault when attempting to call start on the service since the time consumer had not been properly initialized after leaving TIME_SYNC_TOPIC configuration.
- PR 329 & 333: Fixed Carma-clock functionality related to a bug fix in Carma-time-lib when multiple threads call the streets clock singleton method for incoming messages to initialize the clock time. 
- PR 332: Fixed docker builds for all XIL Release CARMA-Streets services.
- PR 333: Fixed TSC service in non-simulation mode which has and exception from the Carma-clock object when trying to update its time to 0. Where this calls to Carma clock is valid only in simulation mode.

### **CARMA-CARLA Integration Tool** 

There are no significant changes to this package as part of the release. Only minor build system and CI issues have been addressed so that this package continues to function.

Enhancements in this release: 

- Issue 28: Add CI workflows for Sonar scan which scans the source code and captures code quality reports on sonar cloud.

Fixes in this release: 

- Issue 27: Fixed Docker file that manually downloads and installs CMake 3.13. This is no longer needed Carma-base ships with Ubuntu's CMake 3.16.x package.
- Issue 31: Fixed Carma-Carla-integration ROS launch errors that shuts down on startup of system.

### **CARMA Time Library** 

The carma-time-lib repository is new repository that adds a library containing logic for managing the real- and simulation time in software that uses it. It provides an interface to useful time functions (such as getting the current time and sleeping until a specified time) that can be swapped between using the system clock and using an externally supplied time source (such as a simulator). This package has been modeled on the ROS time system in terms of general functionality but is able to integrate into software that is not ROS-enabled (such as CARMA Streets and V2X Hub).

Enhancements in this release: 

- PR 3&4: Added CI workflows and setup build arm64. 
- PR 6: Updated Carma-clock class files with time stamp to test in seconds.
- PR 8: Updated Carma-clock method that throws exception when trying to update the time on Realtime clock.
- PR 11: Updated CMake version and Debian packages for focal and Jammy distributions of ubuntu.

Version 4.4.2, released May 10th, 2023
----------------------------------------

### **Summary**
Carma-platform release version 4.4.2 is a hotfix release for 4.4.0.

Fixes in this release:
- PR 2105: Fixed Light Controlled Intersection (LCI) strategic plugin where the node fails to load parameters from the vehicle calibration directory.

Version 4.4.1, released May 9th, 2023
----------------------------------------

### **Summary**
Carma-platform release version 4.4.1 is a hotfix release for 4.4.0.

Fixes in this release:
- Issue 2104: Fixed an exception that can occurs in rare circumstances when the approaching_emergency_vehicle_plugin cannot process all received BSMs from an active ERV at ~10 Hz. Received BSMs that are processed by the approaching_emergency_vehicle_plugin should not result in any exception.

Version 4.4.0, released May 5th, 2023
----------------------------------------

### **Summary**
CARMA system release version 4.4.0 is comprised of new features in CARMA Cloud, CARMA Messenger, and CARMA Platform to support Freight Emergency Response functionality; a newly created Workforce Development (WFD) CDA telematics tool; and CARMA Platform Robotics Operating System 2 (ROS2) upgrades; Along with the above enhancements, several bug fixes and CI related enhancements are included in this release. 

- **<ins>Freight Emergency Response</ins>** – This functionality consists of interactions between automated vehicles equipped with CARMA Platform and a rear-approaching connected emergency response vehicle (ERV) equipped with CARMA Messenger on a highway utilizing cooperative driving automation (CDA). Using CDA, through V2V or V2I communication between the ERV and CARMA equipped vehicles, automated vehicles can detect an approaching ERV on the same route by processing the ERV’s broadcasted BSMs with Part II content, and will attempt to abide by the move-over law by making a lane change and slowing down. When a CARMA equipped vehicle is unable to change lanes out of the approaching ERV’s path, it will broadcast warning messages (EmergencyVehicleResponse) to the ERV until an acknowledgement message (EmergencyVehicleAck) is received.   To enable V2I communication of an ERV’s BSMs to downstream vehicles equipped with CARMA Platform, RSUs equipped with V2X Hub receive ERV BSMs and send them to CARMA Cloud, which identifies V2X Hub instances along the ERV’s future route and forwards the BSMs to those V2X Hubs. Additional details regarding the implemented features within CARMA Platform, CARMA Messenger, and CARMA Cloud to support this new functionality are listed in their respective release notes sections. Potential benefits of this use case are: 

    1. Adoption of CDA in emergency response situations may provide improved and quicker reaction by automated vehicles for smoother transition of states from a normally operating traffic stream to slower traffic movement with less gap availability. These benefits may increase in proportion with higher penetration of technology deployment in the traffic stream. 
    2. CDA technologies could be used to improve reaction time of traffic participants by providing advance notice (through V2V communication) for automated vehicles to respond faster, safely, and more efficiently to the presence of an ERV. 
    3. Through V2I communication, traffic participants responding to the presence of an ERV would have the time to identify optimal and safe gap availability for a safer lane change maneuvering. 
    4. Infrastructure owners and operators (IOOs) participating in the testing and development of CDA on their facilities gain a first-mover advantage and can align the direction of CDA technology with organizational      goals. 
    5.  Organizations that adapt rapidly to new technology, such as CDA, will be better prepared for other rapid technological changes in the transportation field. 

- **<ins>Workforce Development (WFD) CDA-Telematics tool (NEW)</ins>** - This is first release of CDA-Telematics tool. CDA-Telematics Tool is an open-source web-based tool that allows near real-time data collection and streaming from vehicles and infrastructure (entities) for situational awareness during testing or demonstrations. This enables the users to monitor and analyze the behavior of entities via a dynamic and easy-to-use dashboard where users can visualize and plot any data generated from these entities. The tool has both hardware and software components. The hardware includes an edge device or computer connected physically to the entity to collect the data and a cellular network provider to stream the data. The software component includes a data processing server to process the data, a time-series database to store collected data, and a user-interface to visualize and analyze the data. The end-result provides quick and easy analysis of data, collected from an entire fleet and/or region, in near-real-time. 

- **<ins>CARMA Platform Robotics Operating System 2 (ROS2) upgrades</ins>** –  In this release, the ROS2 migration of the CARMA Platform guidance subsystem’s has been completed. As part of this, the ROS1 versions of the basic_autonomy and carma_wm libraries have been removed, as they have been fully replaced by their ROS2 versions in CARMA Platform. 

### **CARMA Platform**

**<ins> Freight Emergency Response Functionalities</ins>**  

Enhancements in this release related to Freight Emergency Response: 

- Issue 2005: Implemented a new approaching_emergency_vehicle_plugin guidance strategic plugin in CARMA Platform, which is responsible for processing received BSMs with Part II content from an approaching ERV (through either direct V2V communication or V2I message forwarding), detecting the time until the approaching ERV will pass the host vehicle, and updating the host vehicle’s maneuver plan accordingly to lane change out of the ERV’s path and/or slow down to a reduced speed while the ERV is actively passing the host vehicle. When an approaching ERV is detected, this plugin sends status messages to the CARMA Web UI to communicate to the user the time until the ERV will pass, and how the host vehicle’s trajectory will be updated in reaction to the approaching ERV. 
- Issue 2085: Hazard lights activation commands are sent to the vehicle when an approaching ERV is in close enough proximity to the host vehicle and in the same lane, which results in the host vehicle being unable to change lanes out of the ERV’s path. 
- Issues 2057 & 2059: Added logic to plan delegator to support turn signal activation on the vehicle and upcoming lane change status updates on the CARMA Web UI. 
- PR 2064: Updated approaching emergency vehicle plugin to publish status messages to the CARMA Web UI based on a detected approaching ERV. 

Known issues in this release related to Freight Emergency Response:  

- Due to the nature of CARMA Platform operating as a proof-of-concept SAE Level 2 system without integrated object detection, emergency response scenarios have not been conducted with an ERV passing a CARMA-equipped CMV in an immediately adjacent lane. All live testing of this functionality for scenarios in which the ERV passes the CMV have been conducted with the ERV passing with at least open lane between itself and the CMV. 

**<ins>ROS2 Upgrades</ins>**

Enhancements in this release related to ROS2 upgrades: 

- Issue 2080: Ported the lci_strategic_plugin guidance strategic plugin to ROS2. 
- Issue 2090: Ported the intersection_transit_maneuvering guidance tactical plugin to ROS2. 
- Issue 2079: Removed ROS1 versions of carma_wm and basic_autonomy that are no longer actively used within CARMA Platform. 
- Issue 2063: Ported lightbar manager library (and its ROS1 dependencies) to ROS2. 

**<ins>Other</ins>** 

Enhancements in this release: 

- Issue 2072: Created a new Stop and Dwell strategic plugin in support of enabling CARMA Platform to detect bus stops and generate stop and wait maneuvers when approaching them.  
- Issue 2076: Implemented a configurable timeout parameter for Plan Delegator when calling a tactical plugin's "plan_trajectory" service to support testing environment using a vector map with longer lanelet lengths (which correlates to longer lane change trajectory generation time). 

Fixes in this release related: 

- Issue 2058: Fixed unit tests in the plan delegator package that are failing. 
- Issue 2075: Lane change trajectory generation takes longer than the 100ms limit (approximately 80-150ms on average for lane change lengths of 70-100 meters) which may cause plan delegator to not publish updated trajectory. 
- PR 2080: Fixed unit tests that are failing for the LCI (Light controlled intersection) strategic plugin package in CARMA Platform. 
- Issue 2093: Fixed Plan Delegator's update Maneuver Parameters function to avoid delay which is approximately 0.3-0.8 seconds for each received maneuver plan. This is a significant amount of delay and can contribute to CARMA control issues, especially at high (30+ mph) speeds. 

### **CDA-Telematics**

The first release includes the several features related to WFD CDA-Telematics tool: 

- The ability to collect (and send in real-time) any data being used in any CARMA system. For CARMA Platform, this data includes but is not limited to: a) current position, speed, acceleration, and steering angle; b) current feature(s) being used; c) moment-by-moment planned trajectories; d) current entities with whom the vehicle is communicating and the data they are receiving. 
- CDA-Telematics tool has ability to collect data from the CARMA Messenger vehicles to get data of vehicles like new speed limits. 
- This telematics tool has ability to capture any data being sent to any other entity and received from any other entity in real-time for CARMA Cloud and CARMA Streets. 
- The CDA-Telematics UI and Grafana is a web-based user interface that allows users to interact with telematic system using login credentials, users can access this telematics web dashboards.  
- CDA-Telematics tool helps the user make it easy to visualize the data and further data processing.  
- The ability to visualize, real-time, on a map, where the vehicle is (e.g., via a moving triangle on a map, pointed in the direction of travel) and some additional real-time information (e.g., speed, via a pop-up box when the vehicle/triangle is clicked on). 
- The ability to plot and edit the plot (e.g., zoom in/out, adjust the axis, etc.) of any two selected variables (i.e., any reported variable can be on the Y-axis, and it may be plotted against any reported variable on the X-axis; the user need only select the variable of interest). The selected variables should be able to show any number of lines, one line for each entity (e.g., vehicle) that the user desires to plot.  
- The ability to turn on/off the viewing/processing of any entity that the user is not interested in. 
- The ability to turn on/off the viewing/processing of any variables that the user is not interested in.  
- The ability to process and visualize any number of vehicles, from all off the world, if they have the Module and a wireless connection.  
- The ability to download the data for further analysis. 

Known issues in this release related to WFD CDA-Telematics tool:  

- Issue 138 & 145: There are two anomalies that the team discovered during integration testing that are documented on Github and the links are provided below. These are not issues as they’ve been fixed but the fix is a workaround due to the limitations that were discovered on InfluxDB (the database that we are using). We have kept them open as anomalies, so we can revisit and try to find a more robust fix for those, after meeting with the InfluxDB development team (third-party).  

### **CARMA Messenger** 

**<ins>Freight Emergency Response Functionalities</ins>**

Enhancements in this release related to Freight Emergency Response: 

- PR 175: Implementation of a new emergency_response_vehicle_plugin to support a connected ERV and enable it to broadcast BSMs that include its current location, speed, emergency lights and sirens status, and future route when applicable. BSM information pertaining to emergency lights, sirens status, and the ERV’s future route are included within the generated BSM’s Part II content. 
- Issue 167: Updated the cpp_message node to enable encoding and decoding of EmergencyVehicleAck messages (NOTE: This functionality is also used within CARMA Platform). 
- Issue 168: Updated the j2735_convertor and cpp_message nodes to enable encoding, decoding, and conversion of BSM Part II content for usage within CARMA Messenger (NOTE: This functionality is also used within CARMA Platform). 
- Issue 169: Updated the cpp_message node to enable encoding and decoding of EmergencyVehicleResponse messages (NOTE: This functionality is also used within CARMA Platform). 
- Issue 173: Creation of a new Emergency Response Vehicle Web UI Widget, which displays relevant ERV information (current speed, location, and future route) to a user, along with warning messages when a downstream vehicle is unable to lane change out of the path of the ERV. 

Fixes in this release related to Freight Emergency Response: 

- Issue 184: Fixed Unit tests that are failing for the CPP message package for tests related to encoding/decoding BSMs, Traffic Control Messages, and Emergency Vehicle Acknowledgement messages. 
- Issue 187: Fixed the /position/velocity topic that does not contain any velocity information published data as it should have current vehicle speed data. 
- Issue 188: Fixed the BSMs published by the emergency response vehicle plugin that doesn’t set properly BSM's presence flag to signify that a regional extension is included. 
- Issues 190 & 191: Fixed Significant delay occurs for high-frequency ROS topics that are bridged from ROS1 to ROS2. 

**<ins>WFD CDA-Telematics Functionalities:</ins>**  

Enhancements in this release related to WFD CDA-Telematics: 

- PR 116: Adds the cyclone DDS configuration xml required to transmit ROS2 data outside the vehicle pc. 
- PR 108: Adds cyclone DDS configuration for the Ford Fusion, Silver Lexus and Blue Lexus. 
- PR 107: Adds cyclone DDS configuration for the Fusion. 
 

### **CARMA Web UI** 

**<ins>Freight Emergency Response Functionalities</ins>**

Enhancements in this release related to Freight Emergency Response: 

- Issue 160: Updated CARMA Web UI to alert the driver of an approaching ERV and the ego vehicle's updated path plan by displaying alert message whether an approaching ERV has been detected, along with the estimated time until the ERV will pass the host vehicle, and the host vehicle’s intended action. 

**<ins>Other</ins>** 

Fixes in this release: 

- Issue 162: Fixed CARMA Web UI which does not subscribe to /guidance/route event and notifications are not displayed to the driver. 
- Issue 151: Fixed base image errors by changing it from Debian Jessie to Debian buster due to Debian Jessie was EOL. 

### **CARMA Cloud**

**<ins>Freight Emergency Response Functionalities</ins>**

Enhancements in this release related to Freight Emergency Response: 

- Issue 45: The creation of a new RSU software package, which enables V2X Hub instances connected to an RSU to register their location information with CARMA Cloud. Additionally, this software package is responsible for processing ERV BSMs received from V2X Hub instances,  identifying other V2X Hub instances connected to RSUs (Roadside Units) along the ERV’s future route, and forwarding the ERV BSMs to those applicable V2X Hubs. 

### **CARMA Torc Pinpoint Driver** 

Enhancements in this release: 

- Issue 35: URDF information should be removed and placed in a vehicle-specific configuration repository. 

### **CARMA Cohda DSRC Driver** 

**<ins>Freight Emergency Response Functionalities</ins>**

Enhancements in this release related to Freight Emergency Response: 

- Issue 111: Driver now supports receiving and broadcasting EmergencyVehicleResponse and EmergencyVehicleAck messages. 

### **CARMA Analytics**  

**<ins>Freight Emergency Response Functionalities</ins>**

Enhancements in this release related to Freight Emergency Response: 

- PR 15: Developed analysis scripts to generate Emergency Response use case verification metrics for V2X Hub and CARMA Cloud. 
- PR 14: Developed analysis scripts to generate Emergency Response use case verification metrics from CARMA Messenger and CARMA Platform rosbags. 

Version 4.3.0, released Feb 10th, 2023
----------------------------------------

### **Summary**
CARMA system release version 4.3.0 is comprised of new features in both CARMA Streets and CARMA Platform to support Adaptive Traffic Signal Optimization in a cooperative driving automation (CDA) environment; a new feature in CARMA Platform to support Cellular Traffic Control Messages; CARMA Platform ROS upgrades; and an upgraded CARMA XiL Co-simulation tool to support CARMA Platform Robotics Operating System 2 (ROS2) implementation. Along with the above enhancements, several bug fixes and CI related enhancements are included in this release. 

- **<ins>Adaptive Traffic Signal Optimization in a CDA environment (TSMO UC3)</ins>** - The TSMO UC3 has developed algorithms to simultaneously optimize signal timing and vehicle trajectories. The algorithms have three main functionalities that have been implemented as multiple new features in both CARMA Streets and CARMA Platform:  

    1. Signal Optimization: CARMA Streets determines a near-optimal signal timing plan based on the received status and intent information from CDA vehicles.
    2. Vehicle Entering Time Estimation: CARMA Streets estimates an entering time to the intersection box for each CDA vehicle.
    3. Trajectory Planning: each CDA vehicle plans a smooth trajectory individually based on the estimated entering time received from CARMA Streets. 
   
  Potential benefits of this use case are: 
  
    1. Higher throughput due to increased entering speeds.
    2. Lower travel delays due to optimized signal timing. 
    3. Lower fuel consumption due to smoother trajectories.  

- **<ins>Integrated Highway Prototype 2 (IHP2) Cellular Traffic Control Messages</ins>** - CARMA Platform is updated to communicate with CARMA Cloud through cellular communication. In this feature, the platform submits a Traffic Control Request (TCR) directly to CARMA Cloud and receives all corresponding Traffic Control Messages (TCMs). As a result, communication with CARMA Cloud is no longer dependent on short range communication (e.g., DSRC or C-V2X) and V2XHub.  Currently CARMA Cloud is only configured to directly communicate with a single entity. The TCM returned from CARMA Cloud over cellular communications may contain a single message or multiple messages. 

### **CARMA Platform**

**<ins>TSMO UC3 Functionalities</ins>** 

To enable the vehicle-side functionalities designed for TSMO UC3,  

   - The World Model feature in CARMA Platform is refactored to enable processing the designed signal phase plan in this use case. 
   - The Light Controlled Intersection (LCI) Strategic Plugin has been refactored to include the new entering time (ET) processing and trajectory selection logics. 

Enhancements in this release related to TSMO UC3 functionalities: 

- Issue 1762: Updated Light Controlled Intersection (LCI) Strategic Plugin so that it uses the scheduled message from the intersection for setting its ET algorithm parameters instead of computing them based on SPAT if the message is available. 
- Issue 1932: Updated Carma world model to support multiple signal groups in a single entry lane. 

Fixes in this release related to TSMO UC3 functionalities: 

- Issue 1986: Fixed light-controlled Intersection tactical plugin exception which cannot store a negative time point in rclcpp time while in ACTIVE state. 
- Issues 1970 and 2012: Fixed CARMA Platform UI incorrect signal head display. 

Known issues in this release related to TSMO UC3 functionalities: 

- While the current LCI logic includes multiple safeguards to prevent a CARMA Platform vehicle from running a red light when engaged with TSMO UC3 logic (Issue 1985, PR 2010), users should exercise caution when deploying and testing TSMO UC3 functionalities. No redlight running was observed during verification testing performed by the Saxton Transportation Laboratory at the west intersection on the campus of Turn Fairbanks Highway Research Center with three CARMA Platform vehicles. Redlight running could still be possible with different roadway geometry, vehicle low-level controller, CARMA Platform tuning parameters, and CARMA Streets configurations. 
- Issue 1996: Fixed Black Pacifica fail to call light-controlled intersection tactical plugin service. Some runs the platform failed to call it the entire run. After turning off camera and Lidar logging, the failure rate reduced to about 10% of the time it fails to call randomly. Currently it is believed to be error in ROS2 service call to components where less computing power could lead to timeouts for the service calls. 
- Issue 2004: Fixed the service call success rate of the LCI (Light Control Intersection) tactical plugin by omitting heavy logic of trajectory generation when unnecessary and uses previous successful trajectory whenever possible but still generates the new trajectory all the time even though it is not used.  
- Issue 2009: Fixed down sampling ratio in light_controlled_intersection_tactical_plugin that caused the platform to crash throwing error such as "Insufficient Spline Points" with new ratio same as in lane cruising plugin's ratio as they use same basic autonomy library functions to generate its trajectory. 

**<ins>IHP2 Functionalities</ins>** 

Enhancements in this release related to IHP2: 

- Issue 1998: Added Opening HTTP tunnels to CARMA Cloud that can be enabled and disabled using a Flag in CARMA Configuration. 
- Issue 1864: Updated Platooning plugin to handle more general cases that involve a single vehicle joining a platoon from an adjacent lane. 
- Issue 1965: Implemented a CARMA Cloud client ROS2 node in CARMA Platform. Functionalities for the CARMA Cloud client ROS node include: 
    - Subscribe to carma_wm_ctrl node to get Traffic Control Request (TCR) data. 
    - Create an HTTP client and sends a TCR post request to Carma-cloud. TCR is in XML format. 
    - Create a j2735_v2x_msgs::msg::Traffic Control Request object and fills it with the data from the XML TCR. 

Fixes in this release related to IHP2: 

- Issue 2022: Fixed an ASN1 mismatch between CARMA Messenger and V2XHub, which caused CARMA Messenger (used within CARMA Platform) to be unable to decode DSRC Traffic Control Messages (TCMs) broadcasted by V2XHub. 

Known issues in this release:  

- Issue 2033: During verification of cellular communication with CARMA Cloud, it was noticed that active event information on the UI do not display accurate information. 

**<ins>ROS2 Upgrades</ins>**

Enhancements in this release related to ROS2 upgrades: 

- Issue 1754: Ported basic autonomy library (and its ROS1 dependencies) to ROS2. 
- PR 1872: Updated port in localization manager node to ROS2 and to launch it from the carma_src.launch.py launch file. 
- Issue 1885: Ported cooperative lane change node from ROS1 to ROS2. 
- Issue 1889: Ported light controlled intersection tactical plugin node to ROS2. 
- Issue 1894: Ported platooning strategic IHP plugin to ROS2. 
- Issue 1904: Ported platooning tactical plugin to ROS2. 
- Issue 1096: Ported SCI strategic plugin to ROS2. 

Fixes in this release related to ROS2 upgrades: 

- Issue 1896: Fixed CARMA UI which cannot connect to the ROS2 network ROS bridge web socket and is being incorrectly launched as a component despite being a python node. 
- Issue 1898: Fixed the ROS2 stop_and_wait_plugin which is failing to load at startup due to the component not being found and wrong binary being loaded as the component. 
- Issue 1899: Fixed ROS2 plugins that are not being configured by the plugin manager at startup of CARMA. 

**<ins>Other</ins>** 

Enhancements and Fixes in this release: 

- Issue 1967: Implemented Simulation testing tooling for launching development environments for testing CARMA Platform in simulation. 
- Issue 1908: Fixed Tactical plugins can take longer than 0.1 seconds to convert maneuver(s) to detailed trajectories which caused the trajectory plan service call from plan delegator to fail. 
- Issue 1911: Fixed vehicle localization when the vehicle is engaged and starts moving to drift out of the lane, but on rviz it shows that the localized position is still in the lane following the path. 
- Issue 1897: Fixed the route following plugin is failing to load into its component container at startup due to a parameter mismatch between a double and integer. 
- Issue 1863: Fixed basic_autonomy library, which was ignoring the Lanelets defined in the “lane_ids” field of received lane follow maneuver messages. 

Known issues in this release:

- Issue 2036: BSM encoding occasionally fails silently in vehicles, allowing them to engage without sending BSMs. 
- Issue 2035: During testing in the CARMA XIL cosimulation tool to evaluate basic vehicle control capabilities, it was observed that CARMA Platform planning and control was unable to complete an initial lane change and right turn in the default CARLA Town4 map. 
- Issue 2034: During testing in the CARMA XIL cosimulation tool to evaluate advanced vehicle control capabilities, it was observed that CARMA Platform Yield Plugin implementation was not aggressive enough in resolving detected conflicts.

### **CARMA-Streets**

**<ins>TSMO UC3 Functionalities</ins>**

To enable the infrastructure-side functionalities designed for TSMO UC3, 

   - The intersection model feature is enhanced to enable correlating the link lanelets and signal group IDs. The intersection model feature enables CARMA Streets to      process both lanelet2 and MAP information for a given intersection in order to understand intersection geometry. 
   - A new service called signal optimization service (SO) is added to CARMA Streets which continuously computes a near-optimal signal phase plan for the near future      based on the status and intent of all vehicles in the area. 
   - A new service called traffic signal controller (TSC) service is added to CARMA Streets. The TSC service in CARMA Streets interfaces with the physical traffic          signal controller to obtain necessary configuration and signal timing and phasing (SPaT) information and command TSC changes based on the optimized signal phase      plan. The TSC Service also produces SPaT data that contains information about planned SPaT changes to be sent out to equipped vehicles. 
   - The existing scheduling service in CARMA Streets is refactored to include the new scheduling logic designed for UC3. The new scheduling logic computes the entry      time for each CDA vehicle based on signal phase plan as well as the status and intent of all vehicles in the area. 

Enhancements in this release related to TSMO UC3 functionalities: 

- Issue 159: Implement UC3 Scheduling logic in CARMA-Streets scheduling library. 
- Issue 166: Added Utility Methods to streets signal phase and timing library for unit translation from J2735. 
- Issue 169: Implement Traffic signal controller state monitoring to generate a state data structure with the required information for all vehicle phases. 
- Issue 170: Update TSC (Traffic signal Controller) service to query signal group to phase mapping information for all vehicle phases including pedestrian phase information for SPaT message population. 
- Issue 173: Update TSC (Traffic Signal Controller) service to receive UDP NTCIP Spat data from the traffic signal controller and convert it into the spat structure provided by the streets signal phase and timing library and publish JSON updates of this Object to a Kafka topic. 
- Issue 179: Added support for retaining start_time for movement_events in the streets_signal_phase library. 
- Issue 197: Updated Consume Desired Phase Plan at TSC Service and populate SPaT MovementEventList information with desired changes. 
- Issue 202: Updated TSC (Traffic Signal Controller) service to broadcast Traffic Signal Controller Configuration for Signal Optimization service. 
- Issue 228: Update TSC Service to make SNMP calls to set phases according to desired phase plan. 
- Issue 219: Updated Signal Optimization Service to consume Traffic Signal Controller (TSC) configuration information from TSC Service on startup to allow it to produce valid desired phase plan messages. 
- Issue 227: Implement Signal Optimization Phase plan modification Algorithm. 
- Issue 247: Added streets signal optimization library to signal opt service as the streets signal optimization library contains the logic for calculating the queue length and dissipation time for each entry lane and finds the list of candidate desired phase plans to pick the one that has the highest delay ratio.  
- Issue 259: Implement logging for performance latency for SPaT generation, SO Desired Phase Plan selection, Intersection Schedule generation, and SNMP dynamic requests. 

Known issues in this release related to TSMO UC3 functionalities: 

- Issue 278: SPaT Get methods returns wrong timestamp at the hour change. 
- Issue 310: Python collect_kafka_logs script will stop consuming messages off of topics when buffer for pipe subprocess is full.  
- Issue 307: Message Service occasionally restarts on single vehicle testing when log level is set to error and vehicle enters intersection.  
- Issue 306: Message Service does purge Mobility Operations messages until it receives Mobility Path and BSM messages.  
- Issue 264: TSC Service throws segfault when Channel Table includes vehicle/pedestrian phase that do not have a Control Source.  
- Issue 263: TSC Service is not pattern aware.  

Other Enhancements and Fixes: 

- Issue 290: Added named Docker volume for MySQL database to allow for V2X-Hub Plugin configurations and users to persist between Docker-compose up and down calls 
- Issue 299: Updated Docker ignore file to ignore log directories for Docker build context to speed up builds. Docker-compose updates to persist Kafka volume between Docker-compose up and down redeployments. 


Version 4.2.0, released July 29th, 2022
----------------------------------------

**Summary:**
Carma-platform release version 4.2.0 is comprised of three major enhancements. First, Cooperative Traffic Management (CTM) Speed harmonization. Second, Cooperative Lane Follow (CLF) - Platoon Formation, Operation, Dissolution. Third, Cooperative Lane Coordination (CLC), cooperative lane merge. Along with the above enhancements, several bug fixes and CI related enhancements are included in this release.

Carma Platform:

Enhancements in this release:

- Issue 1766: Updated port the Traffic Incident Parser node to ROS2 and updated several subscribers and publishers created within carma_wm in order to support publishers being able to re-publish earlier messages to late-joining subscribers.
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
