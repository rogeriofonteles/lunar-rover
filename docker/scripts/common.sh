#!/usr/bin/env bash
# Shared configuration, independent of the clone location and host username.
configure_service() {
    local service="${1:-}"
    case "$service" in
        ros|drake|mujoco) ;;
        *) echo 'Usage: <script> {ros|drake|mujoco}' >&2; return 2 ;;
    esac
    local script_dir
    script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
    export PROJECT_ROOT="$(cd -- "$script_dir/../.." && pwd)"
    export USERNAME="$(id -un)" MY_UID="$(id -u)" MY_GID="$(id -g)"
    local base
    base="$(sed -n 's/^CONTAINER_BASE_NAME:[[:space:]]*//p' "$PROJECT_ROOT/docker/config/project-config.yaml")"
    if [[ ! "$base" =~ ^[a-zA-Z0-9_]+$ ]]; then
        echo 'Invalid CONTAINER_BASE_NAME in docker/config/project-config.yaml' >&2
        return 2
    fi
    export FOLDER_NAME="${base}_${service}"
    export CONTAINER_NAME="${FOLDER_NAME//_/-}"
}
