#!/bin/bash

#  Copyright (C) 2018-2020 LEIDOS.
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


# Build the software and its dependencies

set -ex

source /opt/ros/kinetic/setup.bash
source /opt/autoware.ai/ros/install/setup.bash --extend

cd ~/carma_ws
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
./src/CARMAPlatform/carma_build -c ~/carma_ws -a /opt/autoware.ai/ -x

# Copy the installed files
cd ~/carma_ws 
cp -r install/. /opt/carma/install
chmod -R +x /opt/carma/install 
