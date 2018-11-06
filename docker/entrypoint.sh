#!/bin/bash

# CARMA Docker Entrypoint Script

source /opt/carma/app/bin/setup.bash
tmux new-session -s carma -d -- roslaunch carma saxton_cav.launch mock_can:=true mock_dsrc:=true mock_srx_controller:=true mock_pinpoint:=true mock_radar:=true mock_lateral_controller:=true 