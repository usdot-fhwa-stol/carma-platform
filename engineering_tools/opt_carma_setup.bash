#!/bin/bash

#  Copyright (C) 2018-2024 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# Script sets up the /opt/carma folder for the user
# Takes in two named arguments:
#   --calibration-folder: Path to the vehicle calibration folder (required)
#   --download-maps: Whether to download example maps or not (default: true)

# Default values
DOWNLOAD_MAPS=true
ADD_SIM_FOLDER=false

# Function to display usage message
usage() {
    echo "Usage: $0 --calibration-folder <path> [--download-maps <true|false> --add-sim-folder <false|true>]"
    exit 1
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --calibration-folder)
            CALIBRATION_FOLDER="$2"
            shift 2
            ;;
        --download-maps)
            DOWNLOAD_MAPS="$2"
            shift 2
            ;;
        --add-sim-folder)
            ADD_SIM_FOLDER="$2"
            shift 2
            ;;
        *)
            echo "Unknown parameter passed: $1"
            usage
            ;;
    esac
done

# Ensure calibration folder is provided
if [ -z "$CALIBRATION_FOLDER" ]; then
    echo "Error: --calibration-folder is required."
    usage
fi

# Ensure the calibration folder exists
if [ ! -d "$CALIBRATION_FOLDER" ]; then
    echo "Please specify a valid path to the vehicle calibration folder. Such as carma-config/example_calibration_folder/vehicle/"
    exit -1
fi

GRP_ID=1000
sudo groupadd --gid $GRP_ID carma # create the carma group if it does not already exist and add current user to it
USERNAME=$(whoami)
sudo usermod -a -G $GRP_ID $USERNAME

mkdir -p /opt/carma/logs /opt/carma/.ros /opt/carma/maps /opt/carma/routes /opt/carma/yolo /opt/carma/data

# Check if downloading maps is required
if [ "$DOWNLOAD_MAPS" == "true" ]; then
    FILE_EXT=$RANDOM
    mkdir ~/carma_temp_$FILE_EXT
    cd ~/carma_temp_$FILE_EXT

    echo "Downloading example maps..."
    curl -o /opt/carma/maps/pcd_map.pcd -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-config/develop/example_opt_carma/maps/base_map.pcd

    curl -o /opt/carma/maps/vector_map.osm -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/route/resource/map/town01_vector_map_1.osm

    curl -o /opt/carma/routes/Test_lanelet111_route_1.csv -L  https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/route/resource/route/Test_lanelet111_route_1.csv

    # Clean up temporary files
    rm -R ~/carma_temp_$FILE_EXT
else
    echo "Skipping map downloads..."
fi

sudo chgrp -R $GRP_ID /opt/carma/
sudo chmod 775 -R /opt/carma/

# Link the provided vehicle calibration folder
if [ -L /opt/carma/vehicle ]; then
    sudo rm /opt/carma/vehicle
fi

ln -s "$CALIBRATION_FOLDER" /opt/carma/vehicle

if [ "$ADD_SIM_FOLDER" == "true" ]; then
    echo "Adding folders used for CDASim to be able to save logs"
    mkdir -p /opt/carma/logs/carma_1 /opt/carma/logs/carma_2 /opt/carma-simulation/logs /opt/carma-simulation/scripts /opt/carma-streets/scripts
    sudo chgrp -R $GRP_ID /opt/carma/
    sudo chmod 775 -R /opt/carma/
    sudo chgrp -R $GRP_ID /opt/carma-simulation/
    sudo chmod 775 -R /opt/carma-simulation/
    sudo chgrp -R $GRP_ID /opt/carma-streets/
    sudo chmod 775 -R /opt/carma-streets/
fi
