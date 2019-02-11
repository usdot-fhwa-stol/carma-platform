#!/bin/bash

#  Copyright (C) 2018-2019 LEIDOS.
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

# Intialize the CARMA environment by sourcing the necessary ROS shell scripts
# then run whatever string is passed as argument to this script in that 
# initialized context. E.g. "entrypoint.sh roslaunch carma saxton_cav_docker.launch"
# runs "roslaunch carma saxton_cav_docker.launch" after setting up the environment
# such that ROS and CARMA are on the user's PATH
source ~/.base-image/init-env.sh; exec "$@"
