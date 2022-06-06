#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
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
IMAGE=$(basename `git rev-parse --show-toplevel`)

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
        -d|--develop)
            USERNAME=usdotfhwastoldev
            COMPONENT_VERSION_STRING=develop
            shift
            ;;
        --ros-1-packages|--ros1)
            ROS1_PACKAGES=""
            shift
            ;;
        --ros-2-packages|--ros2)
            ROS2_PACKAGES=""
            shift
            ;;
        *)
            # Var test based on Stack Overflow question: https://stackoverflow.com/questions/5406858/difference-between-unset-and-empty-variables-in-bash
            # Asker: green69
            # Answerer: geekosaur
            if [ "${ROS2_PACKAGES+set}" = "set" ]; then
                ROS2_PACKAGES="$ROS2_PACKAGES $arg"
            elif [ "${ROS1_PACKAGES+set}" = "set" ]; then
                ROS1_PACKAGES="$ROS1_PACKAGES $arg"
            else
                echo "Unknown argument $arg..."
                exit -1
            fi 
            shift
            ;;
    esac
done

if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    echo "Performing incremental build of image to rebuild packages: $ROS1_PACKAGES $ROS2_PACKAGES..."

    echo "Updating Dockerfile references to use most recent image as base image"
    # Trim of docker image LS command sourced from 
    # https://stackoverflow.com/questions/50625619/why-doesnt-the-cut-command-work-for-a-docker-image-ls-command
    # Question Asker: Chris F
    # Question Answerer: Arount
    MOST_RECENT_IMAGE_DATA=$(docker image ls | grep $IMAGE | tr -s ' ')

    if [[ -z "$MOST_RECENT_IMAGE_DATA" ]]; then
        echo No prior image exists to use as base, an initial image must be built first before attempting incremental build.
        exit -1
    fi

    MOST_RECENT_IMAGE_HASH=$(echo $MOST_RECENT_IMAGE_DATA | cut -d " " -f 3)
    MOST_RECENT_IMAGE_ORG=$(echo $MOST_RECENT_IMAGE_DATA | cut -d " " -f 1 | cut -d "/" -f 1)
    MOST_RECENT_IMAGE_TAG=$(echo $MOST_RECENT_IMAGE_DATA | cut -d " " -f 2)
    MOST_RECENT_IMAGE_DATE=$(echo $MOST_RECENT_IMAGE_DATA | cut -d " " -f 4,5,6)

    echo Using $MOST_RECENT_IMAGE_TAG $MOST_RECENT_IMAGE_HASH $MOST_RECENT_IMAGE_DATE as base for partial build...

    sed -i "s|^FROM[[:space:]]*[^[:space:]]*|FROM $MOST_RECENT_IMAGE_HASH|I" ../Dockerfile

    COMPONENT_VERSION_STRING="SNAPSHOT"
    USERNAME="local"
fi

if [[ -z "$COMPONENT_VERSION_STRING" ]]; then
    COMPONENT_VERSION_STRING=$("./get-component-version.sh")
fi

echo "Building docker image for $IMAGE version: $COMPONENT_VERSION_STRING"
echo "Final image name: $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING"

cd ..
if [[ $COMPONENT_VERSION_STRING = "develop" ]]; then
    sed "s|usdotfhwastoldev/|$USERNAME/|g; s|usdotfhwastolcandidate/|$USERNAME/|g; s|usdotfhwastol/|$USERNAME/|g; s|:[0-9]*\.[0-9]*\.[0-9]*|:$COMPONENT_VERSION_STRING|g; s|checkout.bash|checkout.bash -d|g" \
        Dockerfile | docker build -f - --no-cache -t $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING \
        --build-arg VERSION="$COMPONENT_VERSION_STRING" \
        --build-arg VCS_REF=`git rev-parse --short HEAD` \
        --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` .
elif [[ $COMPONENT_VERSION_STRING = "SNAPSHOT" ]]; then
    docker build --network=host --no-cache -t $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING \
        --build-arg ROS1_PACKAGES="$ROS1_PACKAGES" \
        --build-arg ROS2_PACKAGES="$ROS2_PACKAGES" \
        --build-arg VERSION="$COMPONENT_VERSION_STRING" \
        --build-arg VCS_REF=`git rev-parse --short HEAD` \
        --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` .
else
    #The addition of --network=host was a fix for a DNS resolution error that occured 
    #when running the platform inside an Ubuntu 20.04 virtual machine. The error and possible soliutions are 
    # discussed here: https://github.com/moby/moby/issues/41003
    docker build --network=host --no-cache -t $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING \
        --build-arg VERSION="$COMPONENT_VERSION_STRING" \
        --build-arg VCS_REF=`git rev-parse --short HEAD` \
        --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` .
fi

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
