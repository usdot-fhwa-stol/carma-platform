#!/bin/bash
# This script requires that the remote machine and local machine have the same CPU architecture

# Target username and IP address
USERNAME="user"
HOST="192.168.204.129"
TARGET="/home/${USERNAME}/CarmaPlatform"

# Change directory to catkin workspace of this project
PARENT_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd -P)
cd ${PARENT_PATH}
cd ..

# Try catkin_make install in local machine
#exitfn () {
#	trap ERR
#	echo "Catkin_make could not complete the installation"
#}
#trap "exitfn" ERR
#catkin_make install

# Copy all third party libraries
cp devel/lib/libasn1c.so install/lib

# Copy the entire folder to the remote machine
scp -r install ${USERNAME}@${HOST}:/home/${USERNAME}

# SSH into the remote mechine and put install folder to the correct directory
SCRIPT="cp -r /home/${USERNAME}/install ${TARGET}; rm -rf /home/${USERNAME}/install"
ssh -o StrictHostKeyChecking=no -l ${USERNAME} ${HOST} "${SCRIPT}"
