#!/bin/bash

# Bash script to launch the CARMA Platform from source

# Setup environment
source carma.env

# Launch platform
echo "Launching CARMA from source with args ${1}"
roslaunch carma carma_src.launch $1