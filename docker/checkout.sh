#!/bin/bash

# CARMA packages checkout script

echo "Checking out base CARMA platform packages version: ${CARMA_VERSION}..."
cd /root 
mkdir src 
cd src 
git clone git@github.com:fhwa-saxton/CARMASensitive.git --branch $CARMA_VERSION

if [[ ! -z $EXTRA_PACKAGES ]]; then
    echo "Attempting to check out extra CARMA packages from $EXTRA_PACKAGES version $EXTRA_PACKAGES_VERSION..."
    git clone $EXTRA_PACKAGES --branch $EXTRA_PACKAGES_VERSION
else 
    echo "No repository specified for extra CARMA packages to be downloaded from"
fi
