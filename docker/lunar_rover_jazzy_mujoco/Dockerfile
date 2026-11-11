# syntax=docker/dockerfile:1
FROM ubuntu:24.04
ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=developer
ARG MY_UID=1000
ARG MY_GID=1000
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 PATH=/opt/mujoco-env/bin:${PATH} \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
RUN apt-get update && apt-get install --no-install-recommends -y \
    ca-certificates fish python3 python3-venv libgl1 libegl1 libglfw3 \
    libosmesa6 libgl1-mesa-dri xvfb xauth ffmpeg fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv /opt/mujoco-env
COPY mujoco/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt \
    && python -c 'import mujoco; print(mujoco.__version__)'
RUN existing_user="$(getent passwd "${MY_UID}" | cut -d: -f1)" \
    && if [ -n "$existing_user" ]; then userdel "$existing_user"; fi \
    && if ! getent group "${MY_GID}" >/dev/null; then groupadd -g "${MY_GID}" "${USERNAME}"; fi \
    && useradd -m -u "${MY_UID}" -g "${MY_GID}" -s /usr/bin/fish "${USERNAME}"
COPY --chown=${MY_UID}:${MY_GID} mujoco /home/${USERNAME}/mujoco
USER ${USERNAME}
WORKDIR /home/${USERNAME}/mujoco
# Runtime-injected host driver libraries provide GPU OpenGL/EGL and diagnostics.
# Do not install a second NVIDIA driver or force Mesa software rendering here.
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute
CMD ["sleep", "infinity"]
