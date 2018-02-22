#!/bin/bash
# 
### Assumptions
# This script copies the binaries and included files of the carma platform to a remote vehicle
# The script is meant to be run from within a catkin workspace of the form "${catkin_ws}/src/CarmaPlatform"
# The script must be run from within the CarmaPlatform/engineering_tools folder
# This script requires that the remote machine and local machine have the same CPU architecture.
# The script requires that the recieving pc have an example folder called app_v0 which will be duplicated to preserve structure
# The script will attempt to rebuild if no catkin install folder can be found
# All included files needed for launch (not the primary launch file) have been placed in their respective install/share folders at build time
### Usage
# Only deploy new code/html/launch files
# remote_install.bash -n <username> -a
# Redeploy all and rebuild if not install folder exists
# remote_install.bash -n <username>
# Redeploy all and force rebuild
# remote_install.bash -n <username> -b
# Required Options
#	-n Name: The username to use on the target pc
#    Argument: The username
# Additional Options
# -h Host: The ip of the target pc
#    Argument: The ip-address of target pc
# -b Build: A flag which will force a full rebuild
# -p Copy Params: A flag which will cause the params folder to be copied
# -r Copy Routes: A flag which will cause the routes folder to be copied
# -u Copy Urdf: A flag which will cause the urdf folder to be copied
# -e Copy Executables: A flag which will cause the contents of the install directory to be copied
# -l Copy launch: A flag which will cause the carma launch file to be copied
# -c Catkin Workspace: The path of the catkin workspace where the install folder can be found
#    Argument: The path of the catking workspace
# -m Copy Mock Data: A flag which will cause the mock driver data files to be copied
# -a Copy App: A Flag which will cause everything except params, routes, and urdf files to be copied
# -w Copy Web: A Flag which will cause the website (ui) files to be copied
# -s Copy Scripts: A flag which will cause the scripts in engineering_tools to be copied


usage() { echo "Usage: remote_install.bash -n <username> ";}

 

# Default environment variables
HOST="192.168.88.10"
CATKIN_WS="/opt/carma"
BUILD=false
EVERYTHING=true
EXECUTABLES=false
PARAMS=false
ROUTES=false
URDF=false
LAUNCH=false
MOCK_DATA=false
APP=false
WEB=false;
SCRIPTS=false;

while getopts n:h:bpruelmawst:c: option
do
	case "${option}"
	in
		n) USERNAME=${OPTARG};;
		h) HOST=${OPTARG};;
		b) BUILD=true;;
		p) PARAMS=true;;
		r) ROUTES=true;;
		u) URDF=true;;
		e) EXECUTABLES=true;;
		l) LAUNCH=true;;
		c) CATKIN_WS=${OPTARG};;
		m) MOCK_DATA=true;;
		a) APP=true;;
		w) WEB=true;;
		s) SCRIPTS=true;;
		\?) echo "Unknown option: -$OPTARG" >&2; exit 1;;
		:) echo "Missing option argument for -$OPTARG" >&2; exit 1;;
		*) echo "Unimplemented option: -$OPTARG" >&2; exit 1;;

	esac
done

INSTALL_DIR="${CATKIN_WS}/install"

# Move to end of processed options list
shift $(($OPTIND - 1))

# Ensure a username has been specified
if ! ${USERNAME} && [[ -d $1 ]]
then
    echo "Username must be specified with -n option. Example: remote_install.bash -n carma_user" >&2
    exit 1
fi

# Ensure we are in engineering tools
if [ ! ${PWD:(-17)} == "engineering_tools" ]; then
	echo "ERROR: This script was not run from inside engineering tools"
	echo "Cannot located needed files from path: ${LOCAL_CARMA_DIR}"
	echo "Please rerun the script from inside CarmaPlatform/engineering_tools/"
	exit
fi

# Move to parent directory
cd ..
LOCAL_CARMA_DIR=${PWD}

# Show username and IP address of remote machine
echo "Installing to ${HOST} as user: ${USERNAME}..."

# Define paths for files to copy from src
LAUNCH_FILE="${LOCAL_CARMA_DIR}/carmajava/launch/saxton_cav.launch"
SRC_LAUNCH_FILE="${LOCAL_CARMA_DIR}/carmajava/launch/saxton_cav_src.launch"
PARAMS_DIR="${LOCAL_CARMA_DIR}/carmajava/launch/params"
ROUTES_DIR="${LOCAL_CARMA_DIR}/carmajava/route/src/test/resources/routes"
URDF_DIR="${LOCAL_CARMA_DIR}/carmajava/launch/urdf"
MOCK_DATA_DIR="${LOCAL_CARMA_DIR}/carmajava/mock_drivers/src/test/data"
WEBSITE_DIR="${LOCAL_CARMA_DIR}/website"
SCRIPTS_DIR="${LOCAL_CARMA_DIR}/engineering_tools"

# Define paths needed on vehicle pc
CARMA_DIR="/opt/carma"
APP_DIR="${CARMA_DIR}/app"

# If copy executables, params, routes, urdf, or launch is set then don't copy everything
if [ ${EXECUTABLES} == true ] || [ ${PARAMS} == true ] || [ ${ROUTES} == true ] || \
   [ ${URDF} == true ] || [ ${LAUNCH} == true ] || [ ${MOCK_DATA} == true ] || \
	 [ ${APP} == true ] || [ ${WEB} == true ] || [ ${SCRIPTS} == true ]; then
	EVERYTHING=false
fi

# If want to copy all contents which will end up in APP_DIR
if [ ${APP} == true ]; then
	LAUNCH=true
	MOCK_DATA=true
	EXECUTABLES=true
	WEB=true
	SCRIPTS=true
fi

# Script which will set the permissions of all copied files. Starts empty and modified based on options
PERMISSIONS_SCRIPT=""
GROUP="carma"
UG_PERMISSIONS="ug+rwx"
O_PERMISSIONS="o+rx"

# If we want to copy the executables
if [ ${EVERYTHING} == true ] || [ ${EXECUTABLES} == true ]; then
	echo "Trying to copy executables..."
	# Check if we have a current install to copy
	if [ ! -d ${INSTALL_DIR} ]; then
		echo "No install folder exists at: ${INSTALL_DIR} . Initiating a rebuild"
		BUILD=true
	fi

	# If we wish to rebuild
	if [ ${BUILD} == true ]; then
		# Move to catkin_ws
		cd "${CATKIN_WS}"
		# Try catkin_make install in local machine
		catkin_make clean
		exitfn () {
			trap ERR
			echo "Catkin_make could not complete the installation"
		}
		trap "exitfn" ERR
		# Delete old folders for totally clean build
		rm -r build/ devel/ install/
		# Run install
		catkin_make install
	fi

	# Find Version Number
	GUIDANCE_MAVEN="${INSTALL_DIR}/share/maven/gov/dot/fhwa/saxton/carma/guidance"
	cd "${GUIDANCE_MAVEN}" # Go to Maven directory for guidance
	VERSION="$(ls -d */)"
	VERSION=${VERSION::(-1)} # Remove backslash on directory name
	GUIDANCE_JAR="${GUIDANCE_MAVEN}/${VERSION}/guidance-${VERSION}.jar"
	cd "${LOCAL_CARMA_DIR}" # Return to carma directory
	# Extract version number file
	jar xf ${GUIDANCE_JAR} version
	FULL_VERSION_ID="${VERSION}-$(sed -n 1p version)-$(sed -n 2p version)"
	FULL_VERSION_ID="$(echo "${FULL_VERSION_ID}" | tr \( \- | tr -d \) )" # Replace bbad ()
	echo "Version appears to be: ${FULL_VERSION_ID}"

	# Determine target folder
	TARGET="${APP_DIR}_${FULL_VERSION_ID}"
	EXAMPLE_FOLDER="${APP_DIR}_v0"

	# SSH into the remote mechine and create a copy of the app_v0 directory to preserve permissions
	# Then create symlink from app -> app_version
	SCRIPT="rm -r ${TARGET}; cp -r ${EXAMPLE_FOLDER} ${TARGET}; rm -r ${APP_DIR}; ln -s ${TARGET} ${APP_DIR}"
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -l ${USERNAME} ${HOST} "${SCRIPT}"

	# Copy the entire contents of install to the remote machine using current symlink
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${INSTALL_DIR}/." ${USERNAME}@${HOST}:"${APP_DIR}/bin/"
	
	# Update permissions script
	PERMISSIONS_SCRIPT="${PERMISSIONS_SCRIPT}; chgrp -R ${GROUP} ${APP_DIR}; chmod -R ${UG_PERMISSIONS} ${APP_DIR}; chmod -R ${O_PERMISSIONS} ${APP_DIR};"
fi

# If we want to copy params
if [ ${EVERYTHING} == true ] || [ ${PARAMS} == true ]; then
	echo "Trying to copy params"
	# Copy the entire folder to the remote machine
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${PARAMS_DIR}" ${USERNAME}@${HOST}:"${CARMA_DIR}"
	# Update permissions script
	PERMISSIONS_SCRIPT="${PERMISSIONS_SCRIPT}; chgrp -R ${GROUP} ${PARAMS_DIR}; chmod -R ${UG_PERMISSIONS} ${PARAMS_DIR}; chmod -R ${O_PERMISSIONS} ${PARAMS_DIR};"
fi

# If we want to copy routes
if [ ${EVERYTHING} == true ] || [ ${ROUTES} == true ]; then
	echo "Trying to copy routes..."
	# Copy the entire folder to the remote machine
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${ROUTES_DIR}" ${USERNAME}@${HOST}:"${CARMA_DIR}"
		# Update permissions script
	PERMISSIONS_SCRIPT="${PERMISSIONS_SCRIPT}; chgrp -R ${GROUP} ${ROUTES_DIR}; chmod -R ${UG_PERMISSIONS} ${ROUTES_DIR}; chmod -R ${O_PERMISSIONS} ${ROUTES_DIR};"
fi

# If we want to copy urdf
if [ ${EVERYTHING} == true ] || [ ${URDF} == true ]; then
	echo "Trying to copy urdf..."
	# Copy the entire folder to the remote machine
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${URDF_DIR}" ${USERNAME}@${HOST}:"${CARMA_DIR}"
		# Update permissions script
	PERMISSIONS_SCRIPT="${PERMISSIONS_SCRIPT}; chgrp -R ${GROUP} ${URDF_DIR}; chmod -R ${UG_PERMISSIONS} ${URDF_DIR}; chmod -R ${O_PERMISSIONS} ${URDF_DIR};"
fi

# If we want to copy launch file
if [ ${EVERYTHING} == true ] || [ ${LAUNCH} == true ]; then
	echo "Trying to copy launch ..."
	# Copy the launch file to the remote machine using current symlink
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${LAUNCH_FILE}" ${USERNAME}@${HOST}:"${APP_DIR}/launch/"
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${SRC_LAUNCH_FILE}" ${USERNAME}@${HOST}:"${APP_DIR}/launch/"
	# Create symlink to launch file so that roslaunch will work when package is sourced
	SYMLINK_LOCATION="${APP_DIR}/bin/share/carma"
	SCRIPT_1="rm ${SYMLINK_LOCATION}/saxton_cav.launch; ln -s  ${APP_DIR}/launch/saxton_cav.launch ${SYMLINK_LOCATION};"
	SCRIPT_2="rm ${SYMLINK_LOCATION}/saxton_cav_src.launch; ln -s  ${APP_DIR}/launch/saxton_cav_src.launch ${SYMLINK_LOCATION};"
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -l ${USERNAME} ${HOST} "${SCRIPT_1} ${SCRIPT_2}"
		# Update permissions script
	PERMISSIONS_SCRIPT="${PERMISSIONS_SCRIPT}; chgrp -R ${GROUP} ${APP_DIR}/launch; chmod -R ${UG_PERMISSIONS} ${APP_DIR}/launch; chmod -R ${O_PERMISSIONS} ${APP_DIR}/launch;"
fi

# If we want to copy mock data files
if [ ${EVERYTHING} == true ] || [ ${MOCK_DATA} == true ]; then
	echo "Trying to copy mock_data ..."
	# Copy the launch file to the remote machine using current symlink
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${MOCK_DATA_DIR}/." ${USERNAME}@${HOST}:"${APP_DIR}/mock_data/"
	# Update permissions script
	PERMISSIONS_SCRIPT="${PERMISSIONS_SCRIPT}; chgrp -R ${GROUP} ${MOCK_DATA_DIR}; chmod -R ${UG_PERMISSIONS} ${MOCK_DATA_DIR}; chmod -R ${O_PERMISSIONS} ${MOCK_DATA_DIR};"
fi

# If we want to copy website  files
if [ ${EVERYTHING} == true ] || [ ${WEB} == true ]; then
	echo "Trying to copy website ..."
	# Copy the launch file to the remote machine using current symlink
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${WEBSITE_DIR}/." ${USERNAME}@${HOST}:"${APP_DIR}/html/"
	# Update permissions script
	PERMISSIONS_SCRIPT="${PERMISSIONS_SCRIPT}; chgrp -R ${GROUP} ${WEBSITE_DIR}; chmod -R ${UG_PERMISSIONS} ${WEBSITE_DIR}; chmod -R ${O_PERMISSIONS} ${WEBSITE_DIR};"
fi

# If we want to copy mock data files
if [ ${EVERYTHING} == true ] || [ ${SCRIPTS} == true ]; then
	echo "Trying to copy scripts from engineering tools ..."
	# Copy the launch file to the remote machine using current symlink
	scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${SCRIPTS_DIR}" ${USERNAME}@${HOST}:"${APP_DIR}"
	# Update permissions script
	PERMISSIONS_SCRIPT="${PERMISSIONS_SCRIPT}; chgrp -R ${GROUP} ${SCRIPTS_DIR}; chmod -R ${UG_PERMISSIONS} ${SCRIPTS_DIR}; chmod -R ${O_PERMISSIONS} ${SCRIPTS_DIR};"
fi

echo "Setting permissions and sourcing"
ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -l ${USERNAME} ${HOST} "${PERMISSIONS_SCRIPT}"


echo "DONE!"
exit
