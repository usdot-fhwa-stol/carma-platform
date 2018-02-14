#!/bin/bash
# This script requires that the remote machine and local machine have the same CPU architecture.
# This script only supports one kind of project layout: "${catkin_ws}/src/CarmaPlatform"
# The first argument is username for the target remote machine
# The second argument is IP address of the remote machine

# Target username and IP address
USERNAME=$1

HOST="192.168.88.10"
INSTALL_DIR="/opt/carma/install"
BUILD=false
EVERYTHING=true
EXECUTABLES=false
PARAMS=false
ROUTES=false
URDF=false
LAUNCH=false

while getopts h:b:e:p:r:u:e:t:l:i: option
do
	case "${option}"
	in
		h) HOST=${OPTARG};
		b) BUILD=${OPTARG};
		p) PARAMS=${OPTARG};
		r) ROUTES=${OPTARG};
		u) URDF=${OPTARG};
		e) EXECUTABLES=${OPTARG};
		t) TARGET=${OPTARG};
		l) LAUNCH=${OPTARG};
		i) INSTALL_DIR=${OPTARG};
	esac
done

# Move from engineering_tools to CarmaPlatform
cd ../../
LOCAL_CARMA_DIR=${PWD}

if [ ! ${LOCAL_CARMA_DIR:(-13)} == "CarmaPlatform" ]; then
	echo "ERROR: This script was not run from inside engineering tools"
	echo "Cannot located needed files from path: ${LOCAL_CARMA_DIR}"
	echo "Please rerun the script from inside CarmaPlatform/engineering_tools/"
	exit
fi

# Show username and IP address of remote machine
echo "Installing to ${HOST} as user: ${USERNAME}..."

# Define paths for files to copy from src
LAUNCH_FILE="${LOCAL_CARMA_DIR}/carmajava/launch/saxton_cav.launch"
PARAMS_DIR="${LOCAL_CARMA_DIR}/carmajava/launch/params"
ROUTES_DIR="${LOCAL_CARMA_DIR}/carmajava/route/src/test/resources/routefiles"
URDF_DIR="${LOCAL_CARMA_DIR}/carmajava/launch/urdf"

# Define paths needed on vehicle pc
CARMA_DIR="/opt/carma"
APP_DIR="${CARMA_DIR}/app"

# If copy executables, params, routes, urdf, or launch is set then don't copy everything
if [${EXECUTABLES} = true] || [${PARAMS} = true] || [${ROUTES} = true]  || [${URDF} = true] || [${LAUNCH} = true]; then
	EVERYTHING=false
fi

# If we want to copy the executables
if [${EVERYTHING} = true] || [${EXECUTABLES} = true]; then
	echo "Trying to copy executables..."
	# Check if we have a current install to copy
	if [! -d ${INSTALL_DIR}]; then
		echo "No install folder exists at: ${INSTALL_DIR} . Initiating a rebuild"
		BUILD=true
	fi

	# If we wish to rebuild
	if [${BUILD} = true]; then
		# Move to catkin_ws
		cd "../${INSTALL_DIR}"
		# Try catkin_make install in local machine
		catkin_make clean
		exitfn () {
			trap ERR
			echo "Catkin_make could not complete the installation"
		}
		trap "exitfn" ERR
		# Delete old folders for totally clean build
		rm -r build/ devel/
		# Run install
		catkin_make install
	fi

	# Find Version Number
	GUIDANCE_MAVEN="${INSTALL_DIR}/share/maven/gov/dot/fhwa/saxton/carma/guidance"
	VERSION="$(ls -d ${GUIDANCE_MAVEN} \*/)"
	GUIDANCE_JAR="${GUIDANCE_MAVEN}/${VERSION}/guidance-${VERSION}.jar"

	# Extract version number file
	jar xf ${GUIDANCE_JAR} version
	FULL_VERSION_ID="${VERSION}-$(sed -n 1p version)-$(sed -n 2p version)"

	# Determine target folder
	TARGET="${APP_DIR}_${FULL_VERSION_ID}"
	EXAMPLE_FOLDER="${APP_DIR}_v0"

	# SSH into the remote mechine and create a copy of the app_v0 directory to preserve permissions
	# Then create symlink from app -> app_version
	SCRIPT="cp -r ${EXAMPLE_FOLDER} ${TARGET}; ln -s ${TARGET} ${APP_DIR}"
	ssh -o StrictHostKeyChecking=no -l ${USERNAME} ${HOST} "${SCRIPT}"

	# Copy the entire contents of install to the remote machine using current symlink
	scp -r "${INSTALL_DIR}/." ${USERNAME}@${HOST}:"${APP_DIR}/bin/"
fi

# If we want to copy params
if [${EVERYTHING} = true] || [${PARAMS} = true]; then
	echo "Trying to copy params"
	# Copy the entire folder to the remote machine
	scp -r "${PARAMS_DIR}" ${USERNAME}@${HOST}:"${CARMA_DIR}"
fi

# If we want to copy routes
if [${EVERYTHING} = true] || [${ROUTES} = true]; then
	echo "Trying to copy routes..."
	# Copy the entire folder to the remote machine
	scp -r "${ROUTES_DIR}" ${USERNAME}@${HOST}:"${CARMA_DIR}"
fi

# If we want to copy urdf
if [${EVERYTHING} = true] || [${URDF} = true]; then
	echo "Trying to copy urdf..."
	# Copy the entire folder to the remote machine
	scp -r "${URDF_DIR}" ${USERNAME}@${HOST}:"${CARMA_DIR}"
fi

# If we want to copy launch file
if [${EVERYTHING} = true] || [${LAUNCH} = true]; then
	echo "Trying to copy launch ..."
	# Copy the launch file to the remote machine using current symlink
	scp -r "${LAUNCH_FILE}" ${USERNAME}@${HOST}:"${APP_DIR}/launch/"
fi

echo "DONE!"