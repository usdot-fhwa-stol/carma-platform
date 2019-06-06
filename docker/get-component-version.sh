#!/bin/bash

COMPONENT_TAG_PREFIX="${PWD##*/}*"

git describe --match=$COMPONENT_TAG_PREFIX --always --dirty="-SNAPSHOT"

