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

declare -a APT_PACKAGES=()
declare -A APT_SEEN=()

maybe_add_pkg() {
  local pkg="$1"
  if [[ -z "$pkg" ]]; then
    return
  fi
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    return
  fi
  if [[ -n "${APT_SEEN[$pkg]:-}" ]]; then
    return
  fi
  APT_SEEN["$pkg"]=1
  APT_PACKAGES+=("$pkg")
}

if ! python3 -c "import example_interfaces" >/dev/null 2>&1; then
  maybe_add_pkg "ros-${ROS_DISTRO}-example-interfaces"
fi

maybe_add_pkg "ros-${ROS_DISTRO}-rosidl-pycommon"

for impl in ${RMW_IMPLEMENTATION:-} ${MATRIX_RMWS:-}; do
  impl="${impl//[$'\n\r\t']}"
  if [[ -z "$impl" ]]; then
    continue
  fi
  maybe_add_pkg "ros-${ROS_DISTRO}-${impl//_/-}"
done

if ((${#APT_PACKAGES[@]})); then
  echo "[INFO] Installing packages: ${APT_PACKAGES[*]}"
  install_cmd="apt-get update && apt-get install -y --no-install-recommends"
  for pkg in "${APT_PACKAGES[@]}"; do
    install_cmd+=" $pkg"
  done
  flock "$APT_LOCK" bash -c "$install_cmd"
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
