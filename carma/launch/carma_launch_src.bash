#!/bin/bash

# Bash script to launch the CARMA Platform from source

# Setup environment
source carma.env

# Launch platform
echo "Launching CARMA from source with args ${1}"
ros2 launch carma carma_src.launch.py $1
