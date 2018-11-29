#!/bin/bash

docker-compose -f /opt/carma/vehicle/docker-compose.yml down
docker kill carma
docker rm carma
