#!/bin/bash

#  Copyright (C) 2025 LEIDOS.
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


set -exo pipefail

dir=~
BRANCH=develop  # The script will use this unless the -b flag updates it
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -b|--branch)
                  BRANCH=$2
                  shift
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
      esac
done

# Install dependencies for v2x-ros-conversion - needs branch to pull correct debian version
sudo ${dir}/src/v2x-ros-conversion/docker/install_dependencies.sh -b $BRANCH

# Install dependencies for multiple_object_tracking
if [[ -z "$PACKAGES"]]; then
    echo "Installing multiple object tracking dependencies"
    sudo ${dir}/src/multiple_object_tracking/scripts/install_dependencies.sh -b $BRANCH
fi