#!/bin/bash

# Usage
# service_name = [ros, drake]
# ./build_image.sh <service_name>

set -e

export USERNAME=$(whoami)
export MY_UID=$(id -u)

# Parse the config yaml file into env vars
yq -r '. | to_entries | .[] | "\(.key)=\(.value)"' ~/Projects/lunar-rover/docker/config/project-config.yaml > .env
# Source env vars
source .env
# Set folder name according to the required service
FOLDER_NAME="${CONTAINER_BASE_NAME}_$1"
export CONTAINER_NAME=${FOLDER_NAME//_/-}

# mv .env file to the docker-compose dir of the required service and source it
mv ~/Projects/lunar-rover/.env ~/Projects/lunar-rover/docker/$FOLDER_NAME/.env

docker compose -f ~/Projects/lunar-rover/docker/$FOLDER_NAME/docker-compose.build-image.yaml build