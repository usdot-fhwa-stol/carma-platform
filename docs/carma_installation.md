# CARMA Build Workflow

## Build Environment
The CAV Prototype Platform is intended to be built in an Ubuntu 16.04 Xenial Xerus environment installed with ROS Kinetic Kame desktop version. Prebuilt images 
containing a base-line configuration of this system are available upon request. 

1 - Install ROS Kinetic from http://wiki.ros.org/kinetic/Installation/Ubuntu

2-  Install Java
`sudo apt-get install openjdk-8-jdk`

3- Install ROSjava
`sudo apt-get install ros-kinetic-rosjava`

4- Install git
`sudo apt install git`

5- Configure git
`git config --global user.name “..”`
`git config –global user.email “..”`

6- Create a workspace:

`mkdir carma_ws`
`cd carma_ws`


6-1- Clone Autoware Repo

`git clone https://github.com/usdot-fhwa-stol/autoware.ai.git`

6-1-1- Get Autoware prerequisites


`sudo apt-get update`
`sudo apt-get install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin gksu`
`sudo apt-get install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool`
`sudo apt-get install -y libgeographic-dev libpugixml-dev python-catkin-tools libboost-python-dev`
`pip3 install -U setuptools`

6-2- Clone CARMA Packages

`mkdir carma`
`cd carma`
`mkdir src`
`cd src` 

`git clone https://github.com/usdot-fhwa-stol/carma-platform.git`
`git clone https://github.com/usdot-fhwa-stol/carma-msgs.git`
`git clone https://github.com/usdot-fhwa-stol/carma-utils.git`

6-2-1- Install dependencies
`rosdep install -y --from-paths src --ignore-src  --rosdistro $ROS_DISTRO`

9- Build CARMA
`cd ~/carma_ws/carma/src/CARMAPlatform`
`./carma_build –a ~/carma_ws/autoware.ai`
9-1- In case of missing dependencies  install with the command:
`sudo apt-get install ros-kinetic-XXX`


10- Source CARMA
$ source /home/carma_ws/carma/devel/setup.bash