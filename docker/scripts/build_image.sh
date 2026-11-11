#!/usr/bin/env bash
# Usage: ./docker/scripts/build_image.sh {ros|drake|mujoco}
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
configure_service "${1:-}"
docker compose -f "$PROJECT_ROOT/docker/$FOLDER_NAME/docker-compose.build-image.yaml" build --pull
