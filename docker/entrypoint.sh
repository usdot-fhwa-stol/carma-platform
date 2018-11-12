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


# CARMA Docker Entrypoint Script

sudo service apache2 start

source /opt/carma/app/bin/setup.bash
tmux new-session -s carma -- roslaunch carma saxton_cav.launch 
# mock_can:=true mock_dsrc:=true mock_srx_controller:=true mock_pinpoint:=true mock_radar:=true mock_lateral_controller:=true 