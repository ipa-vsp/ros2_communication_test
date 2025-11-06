#!/usr/bin/env python3
"""
Responder container entry point for the ROS 2 QoS matrix benchmark.
Listens for configuration updates and mirrors topic data.
"""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from qos_matrix_common import QoSConfig, decode_profile


class MatrixResponder(Node):
    def __init__(self) -> None:
        super().__init__("matrix_responder")
        self._current_profile = QoSConfig(
            name="default",
            reliability="reliable",
            durability="volatile",
            depth=10,
        )
        self._pong_publisher = None
        self._ping_subscription = None

        self._config_subscription = self.create_subscription(
            String, "matrix/config", self._on_config, 10
        )

        self._apply_topic_profile(self._current_profile)
        self.get_logger().info("Responder ready.")

    def _apply_topic_profile(self, profile: QoSConfig) -> None:
        qos = profile.to_profile()

        if self._ping_subscription:
            self.destroy_subscription(self._ping_subscription)
            self._ping_subscription = None
        if self._pong_publisher:
            self.destroy_publisher(self._pong_publisher)
            self._pong_publisher = None

        self._ping_subscription = self.create_subscription(
            String, "matrix/ping", self._on_ping, qos
        )
        self._pong_publisher = self.create_publisher(String, "matrix/pong", qos)
        self._current_profile = profile
        self.get_logger().info(
            f"Applied topic QoS: reliability={profile.reliability} durability={profile.durability} depth={profile.depth}"
        )

    def _on_ping(self, msg: String) -> None:
        if not self._pong_publisher:
            return
        self._pong_publisher.publish(msg)

    def _on_config(self, msg: String) -> None:
        try:
            profile = decode_profile(msg.data)
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error("Invalid configuration payload: %s", exc)
            return
        self._apply_topic_profile(profile)



def main() -> None:
    rclpy.init()
    responder = MatrixResponder()
    try:
        rclpy.spin(responder)
    except (KeyboardInterrupt, ExternalShutdownException):
        responder.get_logger().info("Responder shutting down.")
    except Exception as exc:  # noqa: BLE001
        if "context is not valid" in str(exc).lower():
            responder.get_logger().info("Responder shutting down.")
        else:
            raise
    finally:
        responder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
