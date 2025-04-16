FROM robotlocomotion/drake:noble

# Install fish shell and clang-format-19
RUN apt-get update && apt-get install -y \
    fish \
    curl \
    sudo \
    nano \
    python3.12-venv \
    clang-format-19 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Build args passed in at build time
ARG USERNAME=rogerio
ARG MY_UID=1000

# Install locales
RUN apt-get update && apt-get install -y locales && rm -rf /var/lib/apt/lists/* \
    && localedef -i en_US -c -f UTF-8 -A /usr/share/locale/locale.alias en_US.UTF-8

ENV LANG=en_US.utf8

# Rename the UID/GID user to USERNAME
RUN existing_user=$(getent passwd "$MY_UID" | cut -d: -f1) && \
    usermod -l $USERNAME -d /home/$USERNAME -m $existing_user && \
    groupmod -n $USERNAME $existing_user && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create group render
RUN groupadd -f render && usermod -aG render rogerio

# Set fish as the default shell for the container's user
RUN chsh -s /usr/bin/fish

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Install Fisher package manager for fish
RUN fish -c "curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && fisher install jorgebucaran/fisher"
# Install bass to use bash commands in fish
RUN fish -c "fisher install edc/bass"