#!/bin/bash

#  Copyright (C) 2021 LEIDOS.
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

# This script is responsable for logging all the ROS Parameters of a running ROS Network
sleep 10 # Sleep for 10s to allow the network to startup

LOG_FOLDER="/opt/carma/logs"

if [ -d "${LOG_FOLDER}/latest" ]
then # If the latest folder exists then log to that
  
  echo "param_dump placing rosparams.yaml in ${LOG_FOLDER}/latest/rosparams.yaml"
  
  rosparam dump > "${LOG_FOLDER}/latest/rosparams.yaml" # Store in file

else # Handle case where latest/ does not exist

  echo "${LOG_FOLDER}/latest not found! param_dump will direct parameter logs to the most recent folder in ${LOG_FOLDER}"

  #  The rest of this else block implementation is based on Stack Overflow answer: https://stackoverflow.com/a/35757377
  #  Asked by Jonathan: https://stackoverflow.com/users/19272/jonathan
  #  Answered by pjh: https://stackoverflow.com/users/4154375/pjh
  #  Credited in accordance with Stack Overflow's CC-BY license
  topdir=${LOG_FOLDER}
  BACKUPDIR=

  # Handle subdirectories beginning with '.', and empty $topdir
  shopt -s dotglob nullglob

  for file in "$topdir"/* ; do
      [[ -L $file || ! -d $file ]] && continue
      [[ -z $BACKUPDIR || $file -nt $BACKUPDIR ]] && BACKUPDIR=$file
  done

  echo "param_dump placing rosparams.yaml in ${BACKUPDIR}"

  rosparam dump > "${BACKUPDIR}/rosparams.yaml" # Store in file
fi

PARAM_DUMP=$(rosparam dump) # Get params
PARAM_DUMP=$(echo ${PARAM_DUMP} | tr ":" '=') # Convert colons to equals to support rostopic pub
PARAM_DUMP=$(echo ${PARAM_DUMP} | tr "'" '"') # Replace single quotes with double to support rostopic pub
PARAM_DUMP=\'${PARAM_DUMP}\' # Wrap whole thing in single quotes

rostopic pub --once /carma_record/rosparams std_msgs/String "data: ${PARAM_DUMP}" # Try to store in rosbag

