#!/usr/bin/env bash
set -euo pipefail

NETWORK_NAME=${NETWORK_NAME:-ros2_matrix}
COMPOSE_FILE=${COMPOSE_FILE:-docker/docker-compose.matrix.yml}

if ! docker network inspect "${NETWORK_NAME}" >/dev/null 2>&1; then
  echo "[INFO] Creating docker network '${NETWORK_NAME}'"
  docker network create "${NETWORK_NAME}"
fi

export ROS2_MATRIX_NETWORK=${NETWORK_NAME}

echo "[INFO] Starting ROS 2 matrix benchmark containers..."
docker compose -f "${COMPOSE_FILE}" \
  --project-name ros2-matrix \
  up --remove-orphans
