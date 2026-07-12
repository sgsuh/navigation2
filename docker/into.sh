#!/bin/bash
# Start the Nav2 Jazzy container (if needed) and open an interactive shell.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.yml"

docker compose -f "$COMPOSE_FILE" up -d
docker compose -f "$COMPOSE_FILE" exec nav2 bash
