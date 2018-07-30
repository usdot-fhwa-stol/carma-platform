The CARMA platform consists of a collection of executable ROS nodes.  Such architecture allows 
the system to be extensible, as the nodes can be distributed across multiple CPU hardware if 
necessary for processing power.  It is built on the Kinetic version of ROS, using ROS Java for 
the majority of the components, while the current set of hardware drivers, the sensor fusion, 
and a couple of static libraries are built in C++.  The system runs on Ubuntu 16.04 on a 64 
bit processor.

Once a set of device drivers is present to interact with the specific host vehicle hardware the 
CARMA system’s Interface Manager will automatically search for and engage the appropriate 
drivers based on their reported device status.  The remainder of the software will function 
independently of the hardware specifics.

Note that in this OSADP submission, some of the existing device driver code has been redacted 
to protect possible proprietary information related to vehicle hardware interfaces.  This driver 
code can probably be provided to interested parties, in a case-by-case basis, by contacting the 
FHWA sponsor of this project.  In the current version, the specific driver directories that have 
been withheld are:  delphi_esr_driver, socketcan_bridge, socketcan_interface, srx_can_driver, 
srx_controller, srx_objects.

CARMA provides a Route component that reads a route definition file, which describes the route 
as a list of successive waypoints, each with several attributes.  This route defines where the 
vehicle is to drive.  Even though CARMA currently is limited to SAE level 1 automation, meaning 
longitudinal control only, it handles lateral information, such as lane position and turning 
requirements, in anticipation of level 2+ capabilities being added in the future.

There is a Sensor Fusion component that takes in data from whatever sensors are active on the 
vehicle and fuses it into a coherent picture of the vehicle’s environment to the best of its 
ability.  The current version fuses forward object radar signals with position and speed data 
from neighbor vehicle Basic Safety Messages (BSMs) to provide a list of known neighbor vehicles 
and their relative locations and speeds, abstracting away the details of how any individual 
sensor is operating or what its data looks like.

Data from the Sensor Fusion primarily feeds the Roadway component, which translates neighboring 
object data, as well as data about the roadbed in the host vehicle’s vicinity, into a complete 
model of the host’s environment, all expressed in terms of the host’s route coordinate system.  
This expression of the environment allows straightforward computations by the Guidance component.

A Message component encodes and decodes radio messages (currently DSRC, per the SAE J2735 2016 
standard) for incoming and outgoing communications with other vehicles and infrastructure entities.

The Guidance component provides all of the intelligence for deciding how the vehicle is to achieve 
its objective (follow the selected route).  It provides facilities for planning a trajectory, 
which is a tactically sized portion of the route (typically 10s to 100s of meters long).  
Trajectories are represented as series of longitudinal and lateral maneuvers.  

Guidance also provides an API for third-party plugins that can use all of the available situational 
information to plan portions of a trajectory.  In fact, trajectories are entirely planned by 
plugins, so CARMA contains several built-in plugins to perform the more fundamental aspects of 
trajectory planning.  These built-in plugins will act as defaults to allow the vehicle to perform 
even if no third-party plugins are present.

Guidance contains an Arbitrator component that orchestrates the various plugins to make their 
contributions to a trajectory according to various priority rules.  Once a trajectory plan is 
complete and validated, Arbitrator passes it on to the Trajectory Executor to read the maneuvers 
and translate them into specific commands for the vehicle to follow.

Guidance also contains components that enable V2V and V2I negotiation functionality.  Negotiation 
is the process of interacting with other entities in the traffic system to enact cooperative 
behavior, attempting to satisfy the needs of each participant.  Typically, negotiations will be 
carried out with DSRC communications using mobility messages created specifically to support 
CARMA negotiations.

The mobility message structure includes four message types, modeled after current J2735 structure, 
and are specifically used for communicating future intent.

Finally, the CARMA platform includes a user interface (UI) that is visible on a web browser.  The 
intent is that the CARMA vehicle has a tablet computer mounted to its dashboard and connected to 
the guidance PC via Wi-Fi.  The PC hosts the web server for this purpose.  The UI allows the 
vehicle’s operator to select a route to drive, select which of the third-party plugins to activate, 
select what information to display on the tablet, and command the vehicle’s automated controls to 
engage.  As an automated route is underway, the UI also displays a variety of status information to 
the operator, some of which is specific to each plugin, and some of which is generic to the system 
as a whole.

Once the CARMA software is installed onto the vehicle’s PC, it can be run from the web UI.  There 
is a small Apache web server set up for the sole purpose of running the CARMA software from the 
tablet.
