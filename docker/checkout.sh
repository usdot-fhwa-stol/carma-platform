#!/bin/bash

# CARMA extra packages checkout script

if [[ ! -z $EXTRA_PACKAGES ]]
    echo "Attempting to check out extra CARMA packages from $EXTRA_PACKAGES..."
    git clone $EXTRA_PACKAGES
else 
    echo "No repository specified for extra CARMA packages to be downloaded from"
fi
