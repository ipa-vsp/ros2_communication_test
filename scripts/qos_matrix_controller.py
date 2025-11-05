#!/usr/bin/env python3
"""
Controller container entry point for the ROS 2 QoS matrix benchmark.
Publishes traffic against the responder and records a compatibility matrix.
"""

from __future__ import annotations

import csv
import json
import math
import os
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from example_interfaces.action import Fibonacci

from qos_matrix_common import (
    QoSConfig,
    encode_profile,
    generate_profiles,
    parse_profiles,
    split_items,
    worst_status,
)


@dataclass(frozen=True)
class TopicLoad:
    hz: float
    payload_bytes: int
    message_count: int

    @property
    def period(self) -> float:
        return 1.0 / self.hz if self.hz > 0 else 0.0


class MatrixController(Node):
    def __init__(
        self,
        profiles: List[QoSConfig],
        loads: List[TopicLoad],
        results_dir: Path,
        warn_latency_ms: float,
        fail_latency_ms: float,
        warn_loss_ratio: float,
        fail_loss_ratio: float,
    ) -> None:
        super().__init__("matrix_controller")
        self._profiles = profiles
        self._loads = loads
        self._results_dir = results_dir
        self._warn_latency_ms = warn_latency_ms
        self._fail_latency_ms = fail_latency_ms
        self._warn_loss_ratio = warn_loss_ratio
        self._fail_loss_ratio = fail_loss_ratio

        self._config_pub = self.create_publisher(String, "matrix/config", 10)
        self._ping_pub = None
        self._pong_sub = None

        self._pending_token: Optional[str] = None
        self._sent_time: float = 0.0
        self._latency_ms: Optional[float] = None
        self._received_token = False

        self._trigger_client = self.create_client(Trigger, "matrix/echo")
        self._action_client = ActionClient(self, Fibonacci, "matrix/fibonacci")

        self._results_dir.mkdir(parents=True, exist_ok=True)

    def apply_profile(self, profile: QoSConfig) -> None:
        qos = profile.to_profile()
        self._rebuild_topic_endpoints(qos)
        self.get_logger().info(
            f"Controller QoS set: reliability={profile.reliability} durability={profile.durability} depth={profile.depth}"
        )

    def send_responder_profile(self, profile: QoSConfig, repeats: int = 3) -> None:
        message = String()
        message.data = encode_profile(profile)
        for _ in range(repeats):
            self._config_pub.publish(message)
            time.sleep(0.1)

    def _rebuild_topic_endpoints(self, qos: QoSProfile) -> None:
        if self._ping_pub:
            self.destroy_publisher(self._ping_pub)
            self._ping_pub = None
        if self._pong_sub:
            self.destroy_subscription(self._pong_sub)
            self._pong_sub = None

        self._ping_pub = self.create_publisher(String, "matrix/ping", qos)
        self._pong_sub = self.create_subscription(String, "matrix/pong", self._on_pong, qos)

    def _on_pong(self, msg: String) -> None:
        if not self._pending_token:
            return
        if "|" not in msg.data:
            return
        token, _ = msg.data.split("|", 1)
        if token != self._pending_token:
            return
        self._latency_ms = (time.monotonic() - self._sent_time) * 1000.0
        self._received_token = True

    def run_matrix(self, executor: MultiThreadedExecutor) -> Dict[str, Dict[str, Dict[str, object]]]:
        matrix: Dict[str, Dict[str, Dict[str, object]]] = {profile.name: {} for profile in self._profiles}
        for controller_profile in self._profiles:
            self.get_logger().info(f"Testing controller profile '{controller_profile.name}'")
            matrix_row = {}
            for responder_profile in self._profiles:
                self.get_logger().info(f" • pairing with responder profile '{responder_profile.name}'")
                cell = self._run_pair(executor, controller_profile, responder_profile)
                matrix_row[responder_profile.name] = cell
            matrix[controller_profile.name] = matrix_row
        return matrix

    def _run_pair(
        self,
        executor: MultiThreadedExecutor,
        controller_profile: QoSConfig,
        responder_profile: QoSConfig,
    ) -> Dict[str, object]:
        self.apply_profile(controller_profile)
        self.send_responder_profile(responder_profile)
        time.sleep(0.5)

        topic_details = []
        topic_statuses: List[str] = []
        for load in self._loads:
            status, details = self._run_topic_load(executor, load)
            topic_details.append(details)
            topic_statuses.append(status)
        topic_status = worst_status(topic_statuses)

        overall = topic_status
        return {
            "status": overall,
            "topic": {
                "status": topic_status,
                "details": topic_details,
            },
        }

    def _run_topic_load(
        self, executor: MultiThreadedExecutor, load: TopicLoad
    ) -> Tuple[str, Dict[str, object]]:
        if not self._ping_pub:
            raise RuntimeError("Topic publisher not initialised.")

        latencies: List[float] = []
        misses = 0
        payload_template = "x" * max(1, load.payload_bytes - 32)
        start_time = time.monotonic()

        for sequence in range(load.message_count):
            token = f"{int(start_time)}-{sequence}"
            payload = f"{token}|{payload_template}"
            message = String()
            message.data = payload

            self._pending_token = token
            self._sent_time = time.monotonic()
            self._received_token = False
            self._latency_ms = None

            self._ping_pub.publish(message)
            deadline = time.monotonic() + max(load.period, 1.0)

            while time.monotonic() < deadline and not self._received_token:
                executor.spin_once(timeout_sec=0.05)

            if self._received_token and self._latency_ms is not None:
                latencies.append(self._latency_ms)
            else:
                misses += 1

            sleep_remaining = load.period - (time.monotonic() - self._sent_time)
            if sleep_remaining > 0:
                time.sleep(sleep_remaining)

        expected = load.message_count
        received = len(latencies)
        loss_ratio = (expected - received) / expected if expected else 0.0
        mean_latency = statistics.mean(latencies) if latencies else math.inf
        max_latency = max(latencies) if latencies else math.inf

        status = "pass"
        if received == 0 or loss_ratio >= self._fail_loss_ratio or max_latency >= self._fail_latency_ms:
            status = "failed"
        elif loss_ratio >= self._warn_loss_ratio or mean_latency >= self._warn_latency_ms:
            status = "caution"

        details = {
            "load_hz": load.hz,
            "payload_bytes": load.payload_bytes,
            "message_count": expected,
            "received": received,
            "loss_ratio": round(loss_ratio, 4),
            "mean_latency_ms": None if math.isinf(mean_latency) else round(mean_latency, 3),
            "max_latency_ms": None if math.isinf(max_latency) else round(max_latency, 3),
            "status": status,
        }
        return status, details

    def _run_service_test(self, executor: MultiThreadedExecutor) -> Tuple[str, Dict[str, object]]:
        request = Trigger.Request()
        future = self._trigger_client.call_async(request)
        deadline = time.monotonic() + self._service_timeout

        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)

        if not future.done():
            return "failed", {"error": "service timeout"}

        response = future.result()
        status = "pass" if response.success else "failed"
        detail = {"message": response.message}
        return status, detail

    def _run_action_test(self, executor: MultiThreadedExecutor) -> Tuple[str, Dict[str, object]]:
        goal = Fibonacci.Goal()
        goal.order = 10
        send_future = self._action_client.send_goal_async(goal)
        deadline = time.monotonic() + self._action_timeout

        while not send_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)

        if not send_future.done():
            return "failed", {"error": "action goal send timeout"}

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return "failed", {"error": "goal rejected"}

        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)

        if not result_future.done():
            return "failed", {"error": "action result timeout"}

        result = result_future.result().result
        sequence = list(result.sequence)
        expected_len = min(10, len(sequence))
        expected_sequence = _fibonacci_sequence(expected_len)
        status = "pass" if sequence[:expected_len] == expected_sequence else "caution"
        detail = {"sequence": sequence}
        return status, detail


def _fibonacci_sequence(n: int) -> List[int]:
    if n <= 0:
        return []
    if n == 1:
        return [0]
    sequence = [0, 1]
    for _ in range(2, n):
        sequence.append(sequence[-1] + sequence[-2])
    return sequence[:n]


def parse_loads(raw: str) -> List[TopicLoad]:
    loads: List[TopicLoad] = []
    for item in (chunk.strip() for chunk in raw.split(",") if chunk.strip()):
        parts = item.split(":")
        if len(parts) not in (2, 3):
            raise ValueError(f"Invalid load specification '{item}'. Use hz:payload[:count].")
        hz = float(parts[0])
        payload = int(parts[1])
        count = int(parts[2]) if len(parts) == 3 else 30
        loads.append(TopicLoad(hz=hz, payload_bytes=payload, message_count=count))
    return loads


def write_results(
    results_dir: Path,
    matrix: Dict[str, Dict[str, Dict[str, object]]],
    profiles: Iterable[QoSConfig],
) -> None:
    status_file = results_dir / "status_matrix.csv"
    json_file = results_dir / "matrix_results.json"

    profile_names = [profile.name for profile in profiles]

    with status_file.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["profile"] + profile_names)
        for row_name in profile_names:
            row_entries = [status_to_icon(matrix[row_name][col]["status"]) for col in profile_names]
            writer.writerow([row_name] + row_entries)

    with json_file.open("w") as handle:
        json.dump(
            {
                "profiles": [profile.as_dict() for profile in profiles],
                "matrix": matrix,
            },
            handle,
            indent=2,
        )


def status_to_icon(status: str) -> str:
    mapping = {
        "pass": "✅",
        "failed": "❌",
        "caution": "⚠️",
    }
    normalized = status.lower()
    return mapping.get(normalized, status)


def main() -> None:
    env_profiles = os.environ.get("MATRIX_PROFILES", "auto")
    env_loads = os.environ.get("MATRIX_LOADS", "10:256:20,50:1024:20")
    results_dir = Path(os.environ.get("RESULTS_DIR", "results/docker_matrix"))

    warn_latency_ms = float(os.environ.get("TOPIC_WARN_LATENCY_MS", 100.0))
    fail_latency_ms = float(os.environ.get("TOPIC_FAIL_LATENCY_MS", 1000.0))
    warn_loss_ratio = float(os.environ.get("TOPIC_WARN_LOSS", 0.1))
    fail_loss_ratio = float(os.environ.get("TOPIC_FAIL_LOSS", 0.3))

    if not env_profiles or env_profiles.strip().lower() == "auto":
        default_distro = os.environ.get("ROS_DISTRO", "humble")
        default_rmw = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        distros = split_items(os.environ.get("MATRIX_DISTROS", default_distro))
        rmws = split_items(os.environ.get("MATRIX_RMWS", default_rmw))
        matrix_reliability = os.environ.get("MATRIX_RELIABILITY", os.environ.get("RELIABILITY", "reliable"))
        matrix_durability = os.environ.get("MATRIX_DURABILITY", os.environ.get("DURABILITY", "volatile"))
        matrix_depth = int(os.environ.get("MATRIX_DEPTH", "10"))
        profiles = generate_profiles(
            distros,
            rmws,
            matrix_reliability,
            matrix_durability,
            depth=matrix_depth,
        )
        if not profiles:
            raise RuntimeError("No profiles generated. Check MATRIX_* environment variables.")
    else:
        profiles = parse_profiles(env_profiles)
    loads = parse_loads(env_loads)

    rclpy.init()
    controller = MatrixController(
        profiles=profiles,
        loads=loads,
        results_dir=results_dir,
        warn_latency_ms=warn_latency_ms,
        fail_latency_ms=fail_latency_ms,
        warn_loss_ratio=warn_loss_ratio,
        fail_loss_ratio=fail_loss_ratio,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        matrix = controller.run_matrix(executor)
        write_results(results_dir, matrix, profiles)
        controller.get_logger().info(f"Matrix results stored in {results_dir}")
    except Exception as exc:  # noqa: BLE001
        controller.get_logger().error(f"Matrix run failed: {exc}")
        raise
    finally:
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
