#!/bin/bash

cd "$(dirname "$0")"
cd ..
git describe --all --match=$COMPONENT_TAG_PREFIX --always --dirty="-SNAPSHOT" | awk -F "/" '{print $NF}'

