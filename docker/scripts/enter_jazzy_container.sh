#!/usr/bin/env bash
# Usage: ./docker/scripts/enter_jazzy_container.sh {ros|drake|mujoco}
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
configure_service "${1:-}"
docker exec -it "$CONTAINER_NAME" fish
