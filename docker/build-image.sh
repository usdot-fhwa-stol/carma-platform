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

cd "$(dirname "$0")"
IMAGE=$(./get-package-name.sh | tr '[:upper:]' '[:lower:]')

echo ""
echo "##### $IMAGE Docker Image Build Script #####"
echo ""

while [[ $# -gt 0 ]]; do
    arg="$1"
    case $arg in
        -v|--version)
            COMPONENT_VERSION_STRING="$2"
            shift
            shift
            ;;
        --system-release)
            SYSTEM_RELEASE=true
            shift
            ;;
        -p|--push)
            PUSH=true
            shift
            ;;
    esac
done

if [[ -z "$COMPONENT_VERSION_STRING" ]]; then
    COMPONENT_VERSION_STRING=$("./get-component-version.sh")
fi

echo "Building docker image for $IMAGE version: $COMPONENT_VERSION_STRING"
echo "Final image name: $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING"

cd ..
docker build --no-cache -t $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING \
    --build-arg VERSION="$COMPONENT_VERSION_STRING" \
    --build-arg VCS_REF=`git rev-parse --short HEAD` \
    --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` .

TAGS=()
TAGS+=("$USERNAME/$IMAGE:$COMPONENT_VERSION_STRING")

docker tag $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING $USERNAME/$IMAGE:latest
TAGS+=("$USERNAME/$IMAGE:latest")

echo "Tagged $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING as $USERNAME/$IMAGE:latest"

if [ "$SYSTEM_RELEASE" = true ]; then
    SYSTEM_VERSION_STRING=$("./get-system-version.sh")
    docker tag $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING $USERNAME/$IMAGE:$SYSTEM_VERSION_STRING
    echo "Tagged $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING as $USERNAME/$IMAGE:$SYSTEM_VERSION_STRING"
    TAGS+=("$USERNAME/$IMAGE:$SYSTEM_VERSION_STRING")
fi

if [ "$PUSH" = true ]; then
    for tag in $TAGS; do
        docker push "${tag}"
    done
fi

echo ""
echo "##### $IMAGE Docker Image Build Done! #####"
