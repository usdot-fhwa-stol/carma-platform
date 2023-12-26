#!/bin/bash
while true; do
inotifywait -e modify,create,delete,move -r /var/lib/docker/volumes/carma-config-data && \
rsync -av --delete /var/lib/docker/volumes/carma-config-data/_data /export/carma_volume

done