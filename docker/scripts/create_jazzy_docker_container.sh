#!/usr/bin/env bash
# Usage: ./docker/scripts/create_jazzy_docker_container.sh {ros|drake|mujoco} [--headless]
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
configure_service "${1:-}"
compose_file="$PROJECT_ROOT/docker/$FOLDER_NAME/docker-compose.create-container.yaml"
if [[ "${2:-}" == --headless ]]; then
    compose_file="$PROJECT_ROOT/docker/$FOLDER_NAME/docker-compose.headless.yaml"
    [[ -f "$compose_file" ]] || { echo 'Headless profile is available for Drake and MuJoCo.' >&2; exit 2; }
elif [[ -n "${2:-}" ]]; then
    echo 'Unknown option; use --headless or omit the second argument.' >&2; exit 2
elif [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null; then
    xhost +si:localuser:"$USERNAME"
fi
docker compose -f "$compose_file" up -d
docker logs -f "$CONTAINER_NAME"
