#!/bin/bash
# Build the Nav2 ROS 2 Jazzy development image.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.yml"

export DOCKER_BUILDKIT=1

docker compose -f "$COMPOSE_FILE" build "$@"
