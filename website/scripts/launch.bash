#!/bin/bash

# Kill all ROS related processes
docker kill carma

# Launch platform
rosBagRecord=$1
CONFIG_FOLDER=/opt/carma/vehicle
docker run \
    --rm \
    --net=host \
    --name carma \
    -v $CONFIG_FOLDER/HostVehicleParams.yaml:/opt/carma/params/HostVehicleParams.yaml \
    -v $CONFIG_FOLDER/saxton_cav.urdf:/opt/carma/urdf/saxton_cav.urdf \
    -v $CONFIG_FOLDER/saxton_cav.launch:/opt/carma/launch/saxton_cav.launch \
    -v $CONFIG_FOLDER/drivers.launch:/opt/carma/launch/drivers.launch \
    -v $CONFIG_FOLDER/carma.config.js:/var/www/html/scripts/carma.config.js \
    -itd carma $rosbagRecord
