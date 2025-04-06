# Allow docker containers to access your X11 session
xhost +local:docker
# Start container (detached)
docker start $1 && docker exec -it $1 fish