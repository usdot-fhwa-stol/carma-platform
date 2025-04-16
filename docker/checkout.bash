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
            *)
                  echo "Unknown argument $1"
                    shift
            ;;
      esac
done

cd "${dir}"/src
git clone --depth=1 https://github.com/usdot-fhwa-stol/multiple_object_tracking --branch "${BRANCH}"


# Install dependencies
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo $SCRIPT_DIR
sudo chmod +x ${SCRIPT_DIR}/install_dependencies.sh
${SCRIPT_DIR}/install_dependencies.sh -b $BRANCH -r $dir