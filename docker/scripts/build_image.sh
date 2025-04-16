#!/bin/bash

# Usage
# service_name = [ros, drake]
# ./build_image.sh <service_name>

set -e

export USERNAME=$(whoami)
export MY_UID=$(id -u)

# Parse the config yaml file into env vars
yq eval '. | to_entries | .[] | "\(.key)=\(.value)"' ~/lunar-rover/docker/config/project-config.yaml > .env
# Source env vars
source .env
# Set folder name according to the required service
FOLDER_NAME="${CONTAINER_BASE_NAME}_$1"

# mv .env file to the docker-compose dir of the required service and source it
mv ~/lunar-rover/.env ~/lunar-rover/docker/$FOLDER_NAME/.env

docker compose -f ~/lunar-rover/docker/$FOLDER_NAME/docker-compose.build-image.yaml build