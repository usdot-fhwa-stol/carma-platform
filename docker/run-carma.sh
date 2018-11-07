#!/bin/bash

#  Copyright (C) 2018 LEIDOS.
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


# Script for running carma via Docker, assumes image carma is already built

CONFIG_FOLDER=$1

docker run \
    --rm \
    -v $CONFIG_FOLDER/HostVehicleParams.yaml:/opt/carma/params/HostVehicleParams.yaml \
    -v $CONFIG_FOLDER/saxton_cav.urdf:/opt/carma/urdf/saxton_cav.urdf \
    -v $CONFIG_FOLDER/saxton_cav.launch:/opt/carma/launch/saxton_cav.launch \
    -v $CONFIG_FOLDER/drivers.launch:/opt/carma/launch/drivers.launch \
    -v $CONFIG_FOLDER/carma.config.js:/var/www/html/scripts/carma.config.js \
    -it carma bash
