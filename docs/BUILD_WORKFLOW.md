# CAV Prototype Build Workflow

## Current Build Status
CircleCI: [![CircleCI](https://circleci.com/gh/fhwa-saxton/CarmaPlatform.svg?style=svg&circle-token=d5faf4fb1cfe6fab581c1e094b84a0dbbe6d3303)](https://circleci.com/gh/fhwa-saxton/CarmaPlatform) 

## Build Environment
The CAV Prototype Platform is intended to be built in an Ubuntu 16.04 Xenial Xerus environment installed with ROS Kinetic Kame desktop version. Prebuilt images 
containing a base-line configuration of this system are available upon request. 

### Dependencies
The following are known dependencies of the CAV Prototype Platform and must becomeinstall before building is possible:

ros-kinetic-desktop-full
ros-kinetic-rosjava

## Catkin Workspace Config
To setup the CAV Prototype Platform build environment you must do the following:

1. Ensure that you have configured your ROS environment by sourcing the appropriate scripts for your shell. 

`source /opt/ros/kinetic/setup.<shell_name>`

If you installed rosjava from source, be sure to also source the appropriate script for your shell

`source <path_to_rosjava>/devel/setup.<shell_name>`

2. Create a Catkin workspace in a location of your choosing and then enter that directory

`mkdir <workspace_name> && cd <workspace_name>`

3. Clone the CarmaPlatform repository into the `src` folder 

`git clone git@github.com:fhwa-saxton/CarmaPlatform.git src`

4. Install the dependencies for the CarmaPlatform project

`rosdep install -i -y --from-paths src/ --rosdistro kinetic`

5. Run the Catkin make procedure 

`catkin_make`

6. Exec the newly generated Catkin build environment script for your shell

`source devel/setup.<shell_name>`

The Catkin build procedure may take some time but after the completion of 6, 
your build environment should be complete.

## Installation
To install the CAV Prototype Platform to your local ROS environment follow the
instructions in the [Catkin Workspace Config](#catkin-workspace-config) section
then run the following command in the root of your Catkin workspace:

`catkin_make install`

## Deployment
TBD

## Configuration
TBD

## Continuous Integration
The CAV Prototype Platform continous integration builds are run at: [CircleCI](https://circleci.com/gh/fhwa-saxton/CarmaPlatform)

