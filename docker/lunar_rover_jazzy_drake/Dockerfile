# syntax=docker/dockerfile:1

# Ubuntu 26.04 is the newest LTS supported by Drake's Python wheels.
FROM ubuntu:26.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=developer
ARG MY_UID=1000
ARG MY_GID=${MY_UID}

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PATH=/opt/drake-env/bin:${PATH}

COPY drake/requirements.txt /tmp/drake-requirements.txt

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        ca-certificates \
        clang-format \
        curl \
        fish \
        ffmpeg \
        g++ \
        git \
        libeigen3-dev \
        libconsole-bridge-dev \
        libglib2.0-0t64 \
        libsm6 \
        libx11-6 \
        nano \
        python3 \
        python3-venv \
        sudo \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv /opt/drake-env \
    && /opt/drake-env/bin/pip install --no-cache-dir --upgrade pip \
    && /opt/drake-env/bin/pip install --no-cache-dir -r /tmp/drake-requirements.txt \
    && python -c "import pydrake; print(f'Drake installed: {pydrake.__file__}')"

# Match the host user so bind-mounted files retain the correct ownership.
RUN existing_user="$(getent passwd "${MY_UID}" | cut -d: -f1)" \
    && if ! getent group "${MY_GID}" >/dev/null; then groupadd -g "${MY_GID}" "${USERNAME}"; fi \
    && if [ -n "${existing_user}" ]; then \
         usermod --login "${USERNAME}" --home "/home/${USERNAME}" --move-home "${existing_user}"; \
       else \
         useradd --uid "${MY_UID}" --gid "${MY_GID}" --create-home --shell /usr/bin/fish "${USERNAME}"; \
       fi \
    && usermod --gid "${MY_GID}" "${USERNAME}" \
    && usermod --append --groups sudo "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && groupadd --force render \
    && usermod --append --groups render,video "${USERNAME}" \
    && chsh --shell /usr/bin/fish "${USERNAME}"

USER ${USERNAME}
WORKDIR /home/${USERNAME}

RUN curl --fail --silent --show-error --location \
        https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish \
        | source=/dev/stdin fish -c 'source $source; fisher install jorgebucaran/fisher edc/bass'

# Match the host's highlighting for recognized commands.
RUN fish -c 'set -U fish_color_command blue'

CMD ["fish"]
