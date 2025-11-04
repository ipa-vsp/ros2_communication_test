#!/usr/bin/env bash
set -euo pipefail

# Runs ROS 2 latency benchmarks across a matrix of QoS settings for each RMW implementation.

DISTROS=${ROS_DISTROS:-"jazzy"}
RMWS=${RMW_IMPLEMENTATIONS:-"rmw_fastrtps_cpp rmw_cyclonedds_cpp"}
RELIABILITIES=${ROS_RELIABILITIES:-"reliable besteffort"}
DURABILITIES=${ROS_DURABILITIES:-"volatile transient_local"}
OUTPUT_ROOT=${QOS_RESULTS_DIR:-"results/qos_matrix"}

mkdir -p "$OUTPUT_ROOT"

for distro in $DISTROS; do
  setup_file="/opt/ros/${distro}/setup.bash"
  if [[ ! -r "$setup_file" ]]; then
    echo "[WARN] Skipping ${distro}: ${setup_file} not found." >&2
    continue
  fi

  for rmw in $RMWS; do
    for reliability in $RELIABILITIES; do
      for durability in $DURABILITIES; do
        result_file="${OUTPUT_ROOT}/${distro}_${rmw}_${reliability}_${durability}.csv"
        echo "[INFO] Running benchmark: distro=${distro}, rmw=${rmw}, reliability=${reliability}, durability=${durability}"
        (
          source "$setup_file"
          export RMW_IMPLEMENTATION="$rmw"
          python3 scripts/ros2_benchmark.py \
            --distro "$distro" \
            --rmw "$rmw" \
            --reliability "$reliability" \
            --durability "$durability" \
            --output "$result_file"
        )
      done
    done
  done
done
