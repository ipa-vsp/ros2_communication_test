#!/usr/bin/env bash
set -euo pipefail

ROLE_ARG=${1:-}
ROLE=${ROLE_ARG:-${ROLE:-controller}}
ROS_DISTRO=${ROS_DISTRO:-humble}

WORKSPACE=${WORKSPACE:-/workspace}

if [[ ! -d "$WORKSPACE" ]]; then
  echo "[ERROR] Workspace directory $WORKSPACE not available" >&2
  exit 1
fi

set +u
: "${AMENT_TRACE_SETUP_FILES:=0}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "$WORKSPACE"

APT_LOCK=${APT_LOCK_PATH:-/tmp/ros2_matrix_apt.lock}
export DEBIAN_FRONTEND=${DEBIAN_FRONTEND:-noninteractive}

if ! python3 -c "import example_interfaces" >/dev/null 2>&1; then
  echo "[INFO] Installing example_interfaces for ${ROS_DISTRO}"
  flock "$APT_LOCK" bash -c "apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-example-interfaces"
fi

case "$ROLE" in
  controller)
    exec python3 scripts/qos_matrix_controller.py
    ;;
  responder)
    exec python3 scripts/qos_matrix_responder.py
    ;;
  *)
    echo "[ERROR] Unknown role: $ROLE" >&2
    exit 1
    ;;
esac
