#!/bin/bash

# Usage
# service_name = [ros, drake]
# ./create_jazzy_docker_container.sh <service_name>

# Parse the config yaml file into env vars
yq -r '. | to_entries | .[] | "\(.key)=\(.value)"' ~/Projects/lunar-rover/docker/config/project-config.yaml > .env
# Source env vars
source .env
# Set folder name according to the required service
export FOLDER_NAME="${CONTAINER_BASE_NAME}_$1"
export CONTAINER_NAME=${FOLDER_NAME//_/-}

# Docker compose will detach from execution
xhost +local:docker
docker compose -f ~/Projects/lunar-rover/docker/$FOLDER_NAME/docker-compose.create-container.yaml up -d
# Visualize container start-up after container was detached
docker logs -f ${CONTAINER_NAME//_/-}