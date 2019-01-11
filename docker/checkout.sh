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

# CARMA packages checkout script

set -ex

if [[ ! -z $EXTRA_PACKAGES ]]; then
    echo "Attempting to check out extra CARMA packages from $EXTRA_PACKAGES version $EXTRA_PACKAGES_VERSION..."
    cd ~/src
    git clone --depth=1 $EXTRA_PACKAGES -b $EXTRA_PACKAGES_VERSION
else 
    echo "No extra packages for CARMA specified."
fi
