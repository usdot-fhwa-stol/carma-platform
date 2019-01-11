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

set -e

USERNAME=usdot-carma
IMAGE=carma

cd "$(dirname "$0")"
cd ../carmajava

echo ""
echo "##### CARMA Docker Image Build Script #####"
echo ""
echo "Getting major, intermediate, minor release numbers..."
GUIDANCE_VERSION="`cat guidance/src/main/java/gov/dot/fhwa/saxton/carma/guidance/CarmaVersion.java`"
MAJOR_RELEASE=$(echo "$GUIDANCE_VERSION" | grep "int major" | grep -o "[0-9]*")
INTERMEDIATE_RELEASE=$(echo "$GUIDANCE_VERSION" | grep "int intermediate" | grep -o "[0-9]*")
MINOR_RELEASE=$(echo "$GUIDANCE_VERSION" | grep "int minor" | grep -o "[0-9]*")
echo "Version: $MAJOR_RELEASE.$INTERMEDIATE_RELEASE.$MINOR_RELEASE"

echo "Getting build number and suffix..."
VER_STRING=`./gradlew -q :guidance:printVersion`
BUILD_NUMBER=`echo $VER_STRING | awk '{ print $1 }'`
BUILD_SUFFIX=`echo $VER_STRING | awk '{ print $2 }'`
echo "Build: $BUILD_NUMBER-$BUILD_SUFFIX"

FULL_VERSION_STRING="$MAJOR_RELEASE.$INTERMEDIATE_RELEASE.$MINOR_RELEASE.$BUILD_NUMBER-$BUILD_SUFFIX"
FULL_VERSION_STRING="$(echo $FULL_VERSION_STRING | sed 's/(.*)\(.*\)/\1/')"

echo "Building docker image for CARMA version: $FULL_VERSION_STRING"
echo "Final image name: $USERNAME/$IMAGE:$FULL_VERSION_STRING"

cd ..
docker build -t $USERNAME/$IMAGE:$FULL_VERSION_STRING --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" --build-arg EXTRA_PACKAGES=git@github.com:fhwa-saxton/CARMASensitive.git .
docker tag $USERNAME/$IMAGE:$FULL_VERSION_STRING $USERNAME/$IMAGE:latest

echo "Tagged $USERNAME/$IMAGE:$FULL_VERSION_STRING as $USERNAME/$IMAGE:latest"

echo ""
echo "##### CARMA Docker Image Build Done! #####"

