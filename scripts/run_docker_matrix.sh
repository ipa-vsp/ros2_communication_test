#!/usr/bin/env bash
set -euo pipefail

# Launches docker-compose driven ROS 2 controller/responder pairs for every
# combination of controller distro/RMW and responder distro/RMW, then rolls the
# results up into distro-specific summary CSVs.

NETWORK_NAME=${NETWORK_NAME:-ros2_matrix}
COMPOSE_FILE=${COMPOSE_FILE:-docker/docker-compose.matrix.yml}
DISTROS_DEFAULT="jazzy" # kilted jazzy iron humble"
DISTROS=${MATRIX_DISTROS:-$DISTROS_DEFAULT}
CONTROLLER_DISTROS=${CONTROLLER_DISTROS:-$DISTROS}
RESPONDER_DISTROS=${RESPONDER_DISTROS:-$DISTROS}
CONTROLLER_RMWS=${CONTROLLER_RMWS:-${MATRIX_RMWS:-"rmw_zenoh_cpp"}}
RESPONDER_RMWS=${RESPONDER_RMWS:-${MATRIX_RMWS:-"rmw_zenoh_cpp"}}
MATRIX_RELIABILITY=${MATRIX_RELIABILITY:-reliable}
MATRIX_DURABILITY=${MATRIX_DURABILITY:-volatile}
MATRIX_DEPTH=${MATRIX_DEPTH:-10}
MATRIX_LOADS=${MATRIX_LOADS:-"10:256:20,50:1024:20"}
RESULTS_ROOT=${RESULTS_ROOT:-results/docker_matrix}

trim() {
  local var="$1"
  var="${var#"${var%%[![:space:]]*}"}"
  var="${var%"${var##*[![:space:]]}"}"
  printf '%s' "$var"
}

sanitize_key() {
  local var
  var=$(trim "$1")
  var="${var//[^A-Za-z0-9_.-]/_}"
  printf '%s' "$var"
}

build_label_key_arrays() {
  local input="$1"
  local -n labels_ref=$2
  local -n keys_ref=$3
  labels_ref=()
  keys_ref=()
  input=$(echo "$input" | tr ',;' ' ')
  for token in $input; do
    local trimmed
    trimmed=$(trim "$token")
    [[ -n "$trimmed" ]] || continue
    labels_ref+=("$trimmed")
    keys_ref+=("$(sanitize_key "$trimmed")")
  done
}

build_value_array() {
  local input="$1"
  local -n array_ref=$2
  array_ref=()
  input=$(echo "$input" | tr ',;' ' ')
  for token in $input; do
    local trimmed
    trimmed=$(trim "$token")
    [[ -n "$trimmed" ]] || continue
    array_ref+=("$trimmed")
  done
}

declare -a CTRL_DISTRO_LABELS=()
declare -a CTRL_DISTRO_KEYS=()
build_label_key_arrays "$CONTROLLER_DISTROS" CTRL_DISTRO_LABELS CTRL_DISTRO_KEYS

declare -a RESP_DISTRO_LABELS=()
declare -a RESP_DISTRO_KEYS=()
build_label_key_arrays "$RESPONDER_DISTROS" RESP_DISTRO_LABELS RESP_DISTRO_KEYS

declare -a CONTROLLER_RMWS_ARRAY=()
declare -a RESPONDER_RMWS_ARRAY=()
build_value_array "$CONTROLLER_RMWS" CONTROLLER_RMWS_ARRAY
build_value_array "$RESPONDER_RMWS" RESPONDER_RMWS_ARRAY

if ((${#CTRL_DISTRO_LABELS[@]} == 0)); then
  echo "[ERROR] No controller ROS 2 distros specified." >&2
  exit 1
fi

if ((${#RESP_DISTRO_LABELS[@]} == 0)); then
  echo "[ERROR] No responder ROS 2 distros specified." >&2
  exit 1
fi

if ((${#CONTROLLER_RMWS_ARRAY[@]} == 0)) || ((${#RESPONDER_RMWS_ARRAY[@]} == 0)); then
  echo "[ERROR] No RMW implementations specified." >&2
  exit 1
fi

mkdir -p "$RESULTS_ROOT"

if ! docker network inspect "${NETWORK_NAME}" >/dev/null 2>&1; then
  echo "[INFO] Creating docker network '${NETWORK_NAME}'"
  docker network create "${NETWORK_NAME}"
fi

cleanup_stack() {
  local project="$1"
  docker compose -f "${COMPOSE_FILE}" --project-name "${project}" down -v >/dev/null 2>&1 || true
}

run_stack() {
  local project="$1"
  shift
  env "$@" docker compose -f "${COMPOSE_FILE}" --project-name "${project}" up \
    --abort-on-container-exit --renew-anon-volumes
  local status=$?
  cleanup_stack "$project"
  return $status
}

aggregate_matrix() {
  local ctrl_label="$1"
  local ctrl_key="$2"
  local summary_file="${RESULTS_ROOT}/${ctrl_key}/summary_status_matrix.csv"
  mkdir -p "$(dirname "$summary_file")"

  {
    printf "profile"
    for resp_idx in "${!RESP_DISTRO_LABELS[@]}"; do
      local resp_label=${RESP_DISTRO_LABELS[$resp_idx]}
      for resp_rmw in "${RESPONDER_RMWS_ARRAY[@]}"; do
        printf ",%s-%s" "$resp_label" "$resp_rmw"
      done
    done
    printf "\n"

    for ctrl_rmw in "${CONTROLLER_RMWS_ARRAY[@]}"; do
      printf "%s-%s" "$ctrl_label" "$ctrl_rmw"
      for resp_idx in "${!RESP_DISTRO_LABELS[@]}"; do
        local resp_key=${RESP_DISTRO_KEYS[$resp_idx]}
        for resp_rmw in "${RESPONDER_RMWS_ARRAY[@]}"; do
          local status_file="${RESULTS_ROOT}/${ctrl_key}/${ctrl_rmw}/${resp_key}/${resp_rmw}/status_matrix.csv"
          local cell="N/A"
          if [[ -f "$status_file" ]]; then
            cell=$(awk -F',' 'NR==2 { gsub(/\r/, "", $2); print $2 }' "$status_file")
          fi
          printf ",%s" "$cell"
        done
      done
      printf "\n"
    done
  } >"$summary_file"
  echo "[INFO] Wrote summary matrix: $summary_file"
}

aggregate_global() {
  local summary_file="${RESULTS_ROOT}/summary_status_matrix.csv"
  mkdir -p "$(dirname "$summary_file")"

  {
    printf "profile"
    for resp_idx in "${!RESP_DISTRO_LABELS[@]}"; do
      local resp_label=${RESP_DISTRO_LABELS[$resp_idx]}
      local resp_key=${RESP_DISTRO_KEYS[$resp_idx]}
      for resp_rmw in "${RESPONDER_RMWS_ARRAY[@]}"; do
        printf ",%s-%s" "$resp_label" "$resp_rmw"
      done
    done
    printf "\n"

    for ctrl_idx in "${!CTRL_DISTRO_LABELS[@]}"; do
      local ctrl_label=${CTRL_DISTRO_LABELS[$ctrl_idx]}
      local ctrl_key=${CTRL_DISTRO_KEYS[$ctrl_idx]}
      for ctrl_rmw in "${CONTROLLER_RMWS_ARRAY[@]}"; do
        printf "%s-%s" "$ctrl_label" "$ctrl_rmw"
        for resp_idx in "${!RESP_DISTRO_LABELS[@]}"; do
          local resp_key=${RESP_DISTRO_KEYS[$resp_idx]}
          for resp_rmw in "${RESPONDER_RMWS_ARRAY[@]}"; do
            local status_file="${RESULTS_ROOT}/${ctrl_key}/${ctrl_rmw}/${resp_key}/${resp_rmw}/status_matrix.csv"
            local cell="N/A"
            if [[ -f "$status_file" ]]; then
              cell=$(awk -F',' 'NR==2 { gsub(/\r/, "", $2); print $2 }' "$status_file")
            fi
            printf ",%s" "$cell"
          done
        done
        printf "\n"
      done
    done
  } >"$summary_file"
  echo "[INFO] Wrote global summary matrix: $summary_file"
}

for ctrl_idx in "${!CTRL_DISTRO_LABELS[@]}"; do
  ctrl_label=${CTRL_DISTRO_LABELS[$ctrl_idx]}
  ctrl_key=${CTRL_DISTRO_KEYS[$ctrl_idx]}
  controller_image="ros:${ctrl_label}-ros-base"

  for ctrl_rmw in "${CONTROLLER_RMWS_ARRAY[@]}"; do
    ctrl_rmw_key=$(sanitize_key "$ctrl_rmw")

    for resp_idx in "${!RESP_DISTRO_LABELS[@]}"; do
      resp_label=${RESP_DISTRO_LABELS[$resp_idx]}
      resp_key=${RESP_DISTRO_KEYS[$resp_idx]}
      responder_image="ros:${resp_label}-ros-base"

      for resp_rmw in "${RESPONDER_RMWS_ARRAY[@]}"; do
        resp_rmw_key=$(sanitize_key "$resp_rmw")

        project="ros2-matrix-${ctrl_key}-${ctrl_rmw_key}-${resp_key}-${resp_rmw_key}"
        run_dir="${RESULTS_ROOT}/${ctrl_key}/${ctrl_rmw}/${resp_key}/${resp_rmw}"
        mkdir -p "$run_dir"

        echo "[INFO] Running matrix cell: controller=${ctrl_label}-${ctrl_rmw}, responder=${resp_label}-${resp_rmw}"
        cleanup_stack "$project"
        if ! run_stack "$project" \
          ROS2_MATRIX_NETWORK="$NETWORK_NAME" \
          ROS_MATRIX_CONTROLLER_IMAGE="$controller_image" \
          ROS_MATRIX_RESPONDER_IMAGE="$responder_image" \
          ZENOH_ROUTER_IMAGE="ros:${ctrl_label}-ros-base" \
          ZENOH_ROUTER_ROS_DISTRO="$ctrl_label" \
          CONTROLLER_ROS_DISTRO="$ctrl_label" \
          RESPONDER_ROS_DISTRO="$resp_label" \
          CONTROLLER_RMW_IMPLEMENTATION="$ctrl_rmw" \
          RESPONDER_RMW_IMPLEMENTATION="$resp_rmw" \
          MATRIX_PROFILES="auto" \
          MATRIX_DISTROS="$ctrl_label" \
          MATRIX_RMWS="$ctrl_rmw $resp_rmw" \
          MATRIX_RELIABILITY="$MATRIX_RELIABILITY" \
          MATRIX_DURABILITY="$MATRIX_DURABILITY" \
          MATRIX_DEPTH="$MATRIX_DEPTH" \
          MATRIX_LOADS="$MATRIX_LOADS" \
          RESULTS_DIR="$run_dir"
        then
          echo "[ERROR] Matrix run failed for controller=${ctrl_rmw}, responder=${resp_rmw} (ctrl distro=${ctrl_label}, resp distro=${resp_label})" >&2
        fi
        cleanup_stack "$project"
      done
    done
  done

  aggregate_matrix "$ctrl_label" "$ctrl_key"
done

aggregate_global
