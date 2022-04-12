#!/bin/sh

INPUT_FILE=$1
OUTPUT_FILENAME=$2

cd "$(dirname "$0")"
touch $OUTPUT_FILENAME
ROSBAG_PATH=$1 OUTPUT_FILENAME=$2 docker-compose up
