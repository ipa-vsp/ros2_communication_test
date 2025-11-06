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

if ! python3 -c "import rosidl_pycommon" >/dev/null 2>&1; then
  case "${ROS_DISTRO}" in
    humble|iron|jazzy)
      maybe_add_pkg "ros-${ROS_DISTRO}-rosidl-runtime-py"
      ;;
    *)
      maybe_add_pkg "ros-${ROS_DISTRO}-rosidl-pycommon"
      ;;
  esac
else
  maybe_add_pkg "ros-${ROS_DISTRO}-rosidl-pycommon"
fi

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

backfill_interface_base_classes() {
  local vendor_file="/workspace/scripts/vendor/rosidl_pycommon/interface_base_classes.py"
  if [[ ! -f "$vendor_file" ]]; then
    return
  fi
  local target_dir=""
  target_dir=$(python3 - <<'PY' 2>/dev/null || true
import pathlib
import rosidl_pycommon
print(pathlib.Path(rosidl_pycommon.__file__).parent)
PY
)
  if [[ -z "$target_dir" ]]; then
    target_dir="/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/rosidl_pycommon"
    mkdir -p "$target_dir"
    if [[ ! -f "$target_dir/__init__.py" ]]; then
      cat >"$target_dir/__init__.py" <<'PY'
from .interface_base_classes import *  # noqa
PY
    fi
  fi
  if [[ -f "$target_dir/interface_base_classes.py" ]]; then
    return
  fi
  echo "[INFO] Installing rosidl_pycommon.interface_base_classes shim."
  cp "$vendor_file" "$target_dir/"
}

backfill_interface_base_classes

ZENOH_ROUTER_PID=""
start_zenoh_router_if_needed() {
  if [[ "${ROLE}" != "controller" ]]; then
    return
  fi
  if [[ -n "${ZENOH_ROUTER_URI:-}" ]]; then
    echo "[INFO] ZENOH_ROUTER_URI provided (${ZENOH_ROUTER_URI}); assuming external zenoh router."
    return
  fi
  if [[ -n "${ZENOH_CONFIG_OVERRIDE:-}" ]]; then
    echo "[INFO] ZENOH_CONFIG_OVERRIDE provided; assuming external zenoh router."
    return
  fi
  if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_zenoh_cpp" ]]; then
    return
  fi
  if pgrep -f "[r]mw_zenohd" >/dev/null 2>&1; then
    echo "[INFO] Zenoh router already running; skipping launch."
    return
  fi
  echo "[INFO] Starting Zenoh router via 'ros2 run rmw_zenoh_cpp rmw_zenohd'."
  ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/zenoh_router.log 2>&1 &
  ZENOH_ROUTER_PID=$!
  trap 'if [[ -n "${ZENOH_ROUTER_PID:-}" ]] && kill -0 "${ZENOH_ROUTER_PID}" 2>/dev/null; then kill "${ZENOH_ROUTER_PID}"; fi' EXIT
  local wait_seconds=${ZENOH_ROUTER_STARTUP_DELAY:-2}
  sleep "$wait_seconds"
  if ! kill -0 "${ZENOH_ROUTER_PID}" 2>/dev/null; then
    echo "[ERROR] Zenoh router failed to start; see /tmp/zenoh_router.log" >&2
    exit 1
  fi
}

start_zenoh_router_if_needed

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
