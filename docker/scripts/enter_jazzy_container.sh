#!/bin/bash

# Usage
# service_name = [ros, drake]
# ./enter_jazzy_container.sh <service_name>

# Parse the config yaml file into env vars
yq -r '. | to_entries | .[] | "\(.key)=\(.value)"' ~/Projects/lunar-rover/docker/config/project-config.yaml > .env
# Source env vars
source .env
# Set folder name according to the required service
CONTAINER_BASE_NAME_DASHED=${CONTAINER_BASE_NAME//_/-}

docker exec -it $CONTAINER_BASE_NAME_DASHED-$1 fish
