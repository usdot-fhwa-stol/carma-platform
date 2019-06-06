#!/bin/bash

SYSTEM_TAG_PREFIX="CARMASystem"

git describe --all --match="$SYSTEM_TAG_PREFIX*" --always --dirty="-SNAPSHOT" | awk -F "/" '{print $NF}'

