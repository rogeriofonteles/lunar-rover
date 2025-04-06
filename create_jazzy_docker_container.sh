#!/bin/bash

sudo mkdir -p /tmp/runtime-rogerio
sudo chown 1001:1001 /tmp/runtime-rogerio

xhost +local:docker
docker run -it \
    --hostname lunar-rover-jazzy \
    --gpus all \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env XDG_RUNTIME_DIR=/tmp/runtime-rogerio \
    --env __NV_PRIME_RENDER_OFFLOAD=1 \
    --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
    --volume /tmp/runtime-rogerio:/tmp/runtime-rogerio \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/lunar-rover/lunar_rover:/home/rogerio/lunar_rover \
    --device /dev/dri \
    --group-add video \
    --group-add render \
    --privileged \
    lunar-rover-jazzy:v0.0.1 \
        fish -c "$(cat <<'FISH'
            if not functions -q _ros2_fish_greeting
                echo "🔧 Installing ros2.fish plugin..."
                bass source /opt/ros/jazzy/setup.bash
                fisher install kpbaks/ros2.fish
            end
            exec fish
FISH
)"