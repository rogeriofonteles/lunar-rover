FROM osrf/ros:jazzy-desktop-full

# Install fish shell, python3-vcstool, and clang-format-19
RUN apt-get update && apt-get install -y \
    fish \
    python3-vcstool \
    clang-format-19 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Build args passed in at build time
ARG USERNAME=rogerio
ARG MY_UID=1000

# Rename the UID/GID user to USERNAME
RUN existing_user=$(getent passwd "$MY_UID" | cut -d: -f1) && \
    usermod -l $USERNAME -d /home/$USERNAME -m $existing_user && \
    groupmod -n $USERNAME $existing_user && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create group render
RUN usermod -aG render rogerio

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Set fish as the default shell for the container's user
RUN sudo chsh -s /usr/bin/fish

# Install Fisher package manager for fish
RUN fish -c "curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && fisher install jorgebucaran/fisher"
# Install bass to use bash commands in fish
RUN fish -c "fisher install edc/bass"

# Install ros deps for controllers
RUN apt-get update && apt-get install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz \
    ros-jazzy-can-msgs \
    ros-jazzy-velocity-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-clearpath-common \
    ros-jazzy-clearpath-platform-msgs \
    ros-jazzy-clearpath-motor-msgs \
    ros-jazzy-clearpath-ros2-socketcan-interface \
    ros-jazzy-serial-driver

# Create the ext folder and chown to USERNAME
RUN mkdir -p /home/$USERNAME/ext/src && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/ext

# Create the clearpath folder and chown to USERNAME
RUN mkdir -p /home/$USERNAME/clearpath && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/clearpath