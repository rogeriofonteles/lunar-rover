export USERNAME=$(whoami)
export UID=$(id -u)
export GID=$(id -g)

docker compose -f ~/lunar-rover/docker/docker-compose-build.yaml build