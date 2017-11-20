#!/bin/bash
# This script requires that the remote machine and local machine have the same CPU architecture.
# This script only supports two kinds of project layouts: "${catkin_ws}/src" & "${catkin_ws}/src/CarmaPlatform"
# The first argument is username for the target remote machine
# The second argument is IP address of the remote machine

# Target username and IP address
USERNAME=$1
HOST=$2
TARGET="/home/${USERNAME}/CarmaPlatform"

# Change directory to catkin workspace of this project
PARENT_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd -P)
cd ${PARENT_PATH}
cd ..
# Go to upper level directory when we have "${catkin_ws}/src/CarmaPlatform" as the project layout
CURRENT_DIRECTORY=$(pwd -P)
ENDS_WITH="src"
if [ ${CURRENT_DIRECTORY:(-3)} == "$ENDS_WITH" ]; then
	cd ..
fi

# Show username and IP address of remote machine
echo "Installing to ${HOST} as user: ${USERNAME}..."

# Try catkin_make install in local machine
catkin_make clean
exitfn () {
	trap ERR
	echo "Catkin_make could not complete the installation"
}
trap "exitfn" ERR
catkin_make install

# Copy all third party libraries
cp devel/lib/libasn1c.so install/lib

# Copy the entire folder to the remote machine
scp -r install ${USERNAME}@${HOST}:/home/${USERNAME}

# SSH into the remote mechine and put install folder to the correct directory
SCRIPT="cp -r /home/${USERNAME}/install ${TARGET}; rm -rf /home/${USERNAME}/install"
ssh -o StrictHostKeyChecking=no -l ${USERNAME} ${HOST} "${SCRIPT}"
