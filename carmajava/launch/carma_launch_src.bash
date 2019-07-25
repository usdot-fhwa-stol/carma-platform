#!/bin/bash

# Bash script to launch the CARMA Platform from source
# Setup environment
set -o allexport
source carma.env
set +o allexport

# Launch platform
echo "Launching CARMA from source with args ${1}"
roslaunch carma carma_src.launch $1