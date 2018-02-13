#!/bin/bash
# This script requires that the remote machine and local machine have the same CPU architecture.
# This script only supports two kinds of project layouts: "${catkin_ws}/src" & "${catkin_ws}/src/CarmaPlatform"
# The first argument is username for the target remote machine
# The second argument is IP address of the remote machine

# Target username and IP address
USERNAME=$1
VERSION=$2
TARGET="/opt/carma/app_$VERSION/bin"

HOST="192.168.88.10"
BUILD=false
EVERYTHING=true
PARAMS=false
ROUTES=false
URDF=false
INSTALL_DIR="/opt/carma/install"

while getopts h:b:e:p:r:u: option
do
	case "${option}"
	in
	h) HOST=${OPTARG};
	b) BUILD=${OPTARG};
	p) PARAMS=${OPTARG};
	r) ROUTES=${OPTARG};
	u) URDF=${OPTARG};
	t) TARGET=${OPTARG};
 esac
done

# Check if we 
if [! -d ${INSTALL_DIR}]; then
	echo "No install folder exists at: $INSTALL_DIR . Initiating a rebuild"
	BUILD=true
fi

if [${BUILD} = true]; then
	# Change directory to catkin workspace of this project
	INSTALL_DIR=$1
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
fi


# Copy the entire folder to the remote machine
scp -r ${INSTALL_DIR} ${USERNAME}@${HOST}:${TARGET}

# SSH into the remote mechine and put install folder to the correct directory
SCRIPT="cp -r /home/${USERNAME}/install ${TARGET}; rm -rf /home/${USERNAME}/install"
ssh -o StrictHostKeyChecking=no -l ${USERNAME} ${HOST} "${SCRIPT}"
