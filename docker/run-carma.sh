#!/bin/bash

# Script for running carma via Docker, assumes image carma is already built
docker run \
    -v $(pwd)/carmajava/launch/cadillac_config/HostVehicleParams.yaml:/opt/carma/params/HostVehicleParams.yaml \
    -v $(pwd)/carmajava/launch/cadillac_config/saxton_cav.urdf:/opt/carma/urdf/saxton_cav.urdf \
    -v $(pwd)/carmajava/launch/cadillac_config/saxton_cav.launch:/opt/carma/launch/saxton_cav.launch \
    -v $(pwd)/carmajava/launch/cadillac_config/drivers.launch:/opt/carma/launch/drivers.launch \
    -v $(pwd)/carmajava/launch/cadillac_config/carma.config.js:/var/www/html/scripts/carma.config.js \
    -it carma bash
