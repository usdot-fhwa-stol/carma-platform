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

# CARMA packages checkout script
# Optional argument to set the root checkout directory with no ending '/' default is '~'

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

cd "${dir}"/src

git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-msgs.git --branch "${BRANCH}"
git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-utils.git --branch "${BRANCH}"
git clone --depth=1 https://github.com/usdot-fhwa-stol/v2x-ros-conversion.git --branch "develop"
git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-message-filters.git --branch "${BRANCH}"
git clone --depth=1 https://github.com/usdot-fhwa-stol/multiple_object_tracking --branch "${BRANCH}"

#rosbridge_suite is a ROS meta-package including all the rosbridge packages.
# NOTE: clone -b flag is used instead of --branch to avoid hook rewriting it
git clone -b ros2 https://github.com/usdot-fhwa-stol/rosbridge_suite

# TODO: Remove V2X-Hub Depedency (CAR-6029)
git clone -b master --depth 1 https://github.com/etherealjoy/qhttpengine.git

git clone -b 7.6.0 --depth 1 --sparse https://github.com/usdot-fhwa-OPS/V2X-Hub.git
cd V2X-Hub
git sparse-checkout set ext/ccserver

# SSC related msgs to record in mcap
cd ${dir}/src
# raptor_dbw_msgs (only ROS2 needed)
git clone --depth 1 --filter=blob:none --sparse https://github.com/NewEagleRaptor/raptor-dbw-ros2.git --depth 1 --branch foxy
cd ${dir}/src/raptor-dbw-ros2
git sparse-checkout init --cone
git sparse-checkout set raptor_dbw_msgs
cd ${dir}/src

# dbw_mkz_msgs (only ROS2 needed)
git clone --depth 1 --filter=blob:none --sparse https://github.com/usdot-fhwa-stol/carma-dbw-mkz-ros.git --depth 1 --branch 1.2.4-ros2
cd ${dir}/src/carma-dbw-mkz-ros
git sparse-checkout init --cone
git sparse-checkout set dbw_mkz_msgs_ros2
cd ${dir}/src

# Install dependencies
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo chmod +x ${SCRIPT_DIR}/install_dependencies.sh
${SCRIPT_DIR}/install_dependencies.sh -b $BRANCH -r $dir
