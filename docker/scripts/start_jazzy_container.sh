#!/usr/bin/env bash
# Usage: ./docker/scripts/start_jazzy_container.sh {ros|drake|mujoco} [entrypoint_file]
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
configure_service "${1:-}"
if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null; then
    xhost +si:localuser:"$USERNAME"
fi
docker start "$CONTAINER_NAME"
if [[ -n "${2:-}" ]]; then
    docker exec -it "$CONTAINER_NAME" fish "$2"
else
    docker exec -it "$CONTAINER_NAME" fish
fi
