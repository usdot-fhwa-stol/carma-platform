#!/bin/bash

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