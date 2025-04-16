#!/bin/bash

# Usage
# service_name = [ros, drake]
# ./start_jazzy_container.sh <service_name> <entrypoint_file>

# Parse the config yaml file into env vars
yq eval '. | to_entries | .[] | "\(.key)=\(.value)"' ~/lunar-rover/docker/config/project-config.yaml > .env
# Source env vars
source .env
# Set folder name according to the required service
CONTAINER_BASE_NAME_DASHED=${CONTAINER_BASE_NAME//_/-}

# Allow docker containers to access your X11 session
xhost +local:docker
# Start container (detached)
docker start $CONTAINER_BASE_NAME_DASHED-$1 && docker exec -it $CONTAINER_BASE_NAME_DASHED-$1 fish $2