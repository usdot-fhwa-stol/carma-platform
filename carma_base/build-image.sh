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
IMAGE=carma-base
CI_IMAGE=$IMAGE-ci

echo ""
echo "##### CARMA Base Docker Image Build Script #####"
echo ""

cd "$(dirname "$0")"


if [[ -z "$1" ]]; then
    TAG=$("../engineering_tools/get-carma-version.sh")
else
    TAG="$1"
fi

echo "Building docker image for CARMA Base version: $TAG"
echo "Final image name: $USERNAME/$IMAGE:$TAG"

docker build --no-cache -t $USERNAME/$IMAGE:$TAG \
    --build-arg VERSION="$TAG" \
    --build-arg VCS_REF=`git rev-parse --short HEAD` \
    --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` .

docker tag $USERNAME/$IMAGE:$TAG $USERNAME/$IMAGE:latest

echo "Tagged $USERNAME/$IMAGE:$TAG as $USERNAME/$IMAGE:latest"

echo "Building docker image for CARMA Base CI version: $TAG"
echo "Final image name: $USERNAME/$CI_IMAGE:$TAG"

docker build -f "../.sonarqube/Dockerfile" --no-cache -t $USERNAME/$CI_IMAGE:$TAG \
    --build-arg VERSION="$TAG" \
    --build-arg VCS_REF=`git rev-parse --short HEAD` \
    --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` .

docker tag $USERNAME/$CI_IMAGE:$TAG $USERNAME/$CI_IMAGE:latest

echo "Tagged $USERNAME/$CI_IMAGE:$TAG as $USERNAME/$CI_IMAGE:latest"

echo ""
echo "##### CARMA Base Docker Image and CI Image Build Done! #####"
