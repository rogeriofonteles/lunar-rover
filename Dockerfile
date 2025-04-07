FROM osrf/ros:jazzy-desktop-full

# Install fish and clang-format
RUN apt-get update && apt-get install -y \
    fish \
    clang-format-19 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ros deps for controllers
RUN apt-get update && apt-get install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz \
    ros-jazzy-velocity-controllers \
    ros-jazzy-joint-trajectory-controller

# Create user
ARG USERNAME=rogerio
RUN useradd -m -s /usr/bin/fish $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create group render
RUN groupadd -g 992 render && usermod -aG render rogerio

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME    

# Set fish as the default shell for the container's user
RUN sudo chsh -s /usr/bin/fish

# Install Fisher package manager for fish
RUN fish -c "curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && fisher install jorgebucaran/fisher"
# Install bass to use bash commands in fish
RUN fish -c "fisher install edc/bass"

# Set the default command to fish shell
CMD ["fish"]