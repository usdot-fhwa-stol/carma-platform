#!/bin/bash

# Copyright (C) 2018-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.


# Docker installation script

# Ensure docker group id is 999. CARMA UI is dependent on having the group id be the same between the image and vehicle PC.
if [[ -z $(grep  -i "docker" /etc/group) ]]; then
    	echo "User docker does not exists in /etc/group, creating docker group id of 999"
   	    addgroup --gid 999 docker
else
	echo "User docker already exists in /etc/group."

	if [[ $(grep  -i "docker" /etc/group) == *"docker:x:999"* ]]; then
	    echo "Docker group id is correct"
	else
	    echo "ERROR: CARMA requires the docker group id 999 in the host PC. Please update the Host PC to correct this before trying again."
	    exit
	fi
fi

# Compiled from https://docs.docker.com/install/linux/docker-ce/ubuntu/

sudo apt-get remove docker docker-engine docker.io
sudo apt-get update
sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install -y docker-ce docker-compose
sudo usermod -a -G docker $USER

echo "Docker installation complete, please log out and back in again to use Docker!"