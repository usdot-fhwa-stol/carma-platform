#!/bin/bash

COMPONENT_TAG_PREFIX=`./get-package-name.sh`
git describe --all --match="$COMPONENT_TAG_PREFIX_*" --always --dirty="-SNAPSHOT" | awk -F "/" '{print $NF}' | sed "s/$COMPONENT_TAG_PREFIX\_//"

