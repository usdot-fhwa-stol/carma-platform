#!/bin/bash

cd "$(dirname "$0")"
cd ..
COMPONENT_TAG_PREFIX="${PWD##*/}"
git describe --all --match="$COMPONENT_TAG_PREFIX*" --always --dirty="-SNAPSHOT" | awk -F "/" '{print $NF}'

