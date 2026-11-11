FROM osrf/ros:jazzy-desktop-full

# Install fish shell, python3-vcstool, and clang-format-19
RUN apt-get update && apt-get install -y \
    fish \
    curl \
    sudo \
    python3-vcstool \
    clang-format-19 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ros deps for controllers and simulation
RUN apt-get update && apt-get install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz \
    ros-jazzy-can-msgs \
    ros-jazzy-velocity-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-serial-driver \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher

# Build args passed in at build time
ARG USERNAME=developer
ARG MY_UID=1000
ARG MY_GID=1000

# Match host identity without assuming an existing user or a fixed username.
RUN existing_user="$(getent passwd "${MY_UID}" | cut -d: -f1)" \
    && if ! getent group "${MY_GID}" >/dev/null; then groupadd -g "${MY_GID}" "${USERNAME}"; fi \
    && if [ -n "${existing_user}" ]; then \
         usermod --login "${USERNAME}" --home "/home/${USERNAME}" --move-home "${existing_user}"; \
       else \
         useradd -m -u "${MY_UID}" -g "${MY_GID}" "${USERNAME}"; \
       fi \
    && usermod -g "${MY_GID}" -aG sudo "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && groupadd -f render \
    && usermod -aG render,video "${USERNAME}" \
    && chsh -s /usr/bin/fish "${USERNAME}"

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Create the external workspace as the container user.
RUN mkdir -p /home/$USERNAME/ext/src


# Install shell plugins for the actual container user.
RUN curl --fail --silent --show-error --location \
        https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish \
        | source=/dev/stdin fish -c 'source $source; fisher install jorgebucaran/fisher edc/bass'
