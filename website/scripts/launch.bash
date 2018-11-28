#!/bin/bash

# Kill all CARMA containers
docker-compose -f /opt/carma/vehicle/docker-compose.yml down

# Launch platform
docker-compose -f /opt/carma/vehicle/docker-compose.yml up -d
