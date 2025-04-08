#!/bin/bash

# Docker compose will detach from execution
# To visualize the execution you can use `docker logs -f <container_id>`
xhost +local:docker
docker compose -f ~/lunar-rover/docker/docker-compose-create.yaml up -d