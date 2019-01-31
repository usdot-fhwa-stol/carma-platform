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

USERNAME=usdotfhwastol
IMAGE=carma-config
cd "$(dirname "$0")"
DIR_NAME=${PWD##*/}
CONFIG_NAME=`echo $DIR_NAME | sed 's/_/-/g'`

echo ""
echo "##### CARMA $CONFIG_NAME Configuration Docker Image Build Script #####"
echo ""


TAG="$("../../engineering_tools/get-carma-version.sh")-$CONFIG_NAME"

echo "Building docker image for CARMA Configuration version: $TAG"
echo "Final image name: $USERNAME/$IMAGE:$TAG"

docker build --no-cache -t $USERNAME/$IMAGE:$TAG \
    --build-arg VERSION="$TAG" \
    --build-arg VCS_REF=`git rev-parse --short HEAD` \
    --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` .

echo ""
echo "##### CARMA $CONFIG_NAME Docker Image Build Done! #####"
