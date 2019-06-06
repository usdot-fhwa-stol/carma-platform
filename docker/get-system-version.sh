#!/bin/bash

COMPONENT_TAG_PREFIX="CARMAVersion*"

git describe --match=$COMPONENT_TAG_PREFIX --always --dirty="-SNAPSHOT"

