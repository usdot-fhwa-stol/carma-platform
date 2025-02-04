#!/bin/bash

#  Copyright (C) 2018-2023 LEIDOS.
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

set -e

# Build the software and its dependencies

cd ~/carma_ws

if [[ -z "$PACKAGES" ]]; then
    echo "Installing multiple object tracking dependencies"
    sudo ./src/multiple_object_tracking/scripts/install_dependencies.sh
fi

# Install stol-j2735-1 package for cpp_message
. /etc/lsb-release
# add the STOL APT repository
echo "deb [trusted=yes] http://s3.amazonaws.com/stol-apt-repository ${DISTRIB_CODENAME} main" | sudo tee /etc/apt/sources.list.d/stol-apt-repository.list
# install carma-j2735 library for encoding/decoding of messages
sudo apt-get update
sudo apt-get install -y stol-j2735-1

sudo mkdir -p /opt/carma # Create install directory
sudo chown carma /opt/carma # Set owner to expose permissions for build
sudo chgrp carma /opt/carma # Set group to expose permissions for build

# Source the autoware installation

if [[ ! -z "$PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install/setup.bash
else
    echo "Sourcing base image for full build..."
    source /opt/autoware.ai/ros/install/setup.bash
fi

cd ~/carma_ws

echo "Building Selected CARMA Components"

if [[ ! -z "$PACKAGES" ]]; then
    echo "Incrementally building following packages and those dependent on them: $PACKAGES"
    colcon build --install-base /opt/carma/install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $PACKAGES
else
    echo "Building all CARMA components..."
    colcon build  --install-base /opt/carma/install --build-base build --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

echo "Build of CARMA Components Complete"
