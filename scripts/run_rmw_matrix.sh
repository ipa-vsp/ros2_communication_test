#!/usr/bin/env bash
set -euo pipefail

# Runs ROS 2 latency benchmarks across a matrix of distributions and RMW implementations.

DISTROS=${ROS_DISTROS:-"jazzy humble"}
RMWS=${RMW_IMPLEMENTATIONS:-"rmw_fastrtps_cpp rmw_cyclonedds_cpp"}
OUTPUT_ROOT=${RMW_RESULTS_DIR:-"results/rmw_matrix"}

mkdir -p "$OUTPUT_ROOT"

for distro in $DISTROS; do
  setup_file="/opt/ros/${distro}/setup.bash"
  if [[ ! -r "$setup_file" ]]; then
    echo "[WARN] Skipping ${distro}: ${setup_file} not found." >&2
    continue
  fi

  for rmw in $RMWS; do
    result_file="${OUTPUT_ROOT}/${distro}_${rmw}_reliable_volatile.csv"
    echo "[INFO] Running benchmark: distro=${distro}, rmw=${rmw}"
    (
      source "$setup_file"
      export RMW_IMPLEMENTATION="$rmw"
      python3 scripts/ros2_benchmark.py \
        --distro "$distro" \
        --rmw "$rmw" \
        --reliability reliable \
        --durability volatile \
        --output "$result_file"
    )
  done
done
