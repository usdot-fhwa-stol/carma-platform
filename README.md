# CarmaPlatform
This is the second generation, reusable CARMA platform developed under STOL II, Task Order 13.

## Build Status
CircleCI: [![CircleCI](https://circleci.com/gh/fhwa-saxton/CarmaPlatform.svg?style=svg&circle-token=d5faf4fb1cfe6fab581c1e094b84a0dbbe6d3303)](https://circleci.com/gh/fhwa-saxton/CarmaPlatform) 

## Build Instructions:
To setup the CAV Prototype Platform build environment you must do the following:

1. Ensure that you have configured your ROS environment by sourcing the appropriate scripts for your shell. 

`source /opt/ros/kinetic/setup.<SHELL_NAME>`

2. Create a Catkin workspace in a location of your choosing and then enter that directory

`mkdir <workspace_name> && cd <workspace_name>`

3. Clone the CarmaPlatform repository into the `src` folder 

`git clone git@github.com:fhwa-saxton/CarmaPlatform.git src`

4. Install the dependencies for the CarmaPlatform project

`rosdep install -i -y --from-paths src/ --rosdistro kinetic`

5. Run the Catkin make procedure 

`catkin_make`

6. Exec the newly generated Catkin build environment script for your shell

`source devel/setup.<SHELL_NAME>`

The Catkin build procedure may take some time but after the completion of 6, 
your build environment should be complete.

For more instructions on building and installing, please read [BUILD_WORKFLOW.md](docs/BUILD_WORKFLOW.md)

