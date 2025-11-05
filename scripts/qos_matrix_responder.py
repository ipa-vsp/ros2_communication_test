#!/usr/bin/env python3
"""
Responder container entry point for the ROS 2 QoS matrix benchmark.
Listens for configuration updates, mirrors topic data, and hosts a service/action pair.
"""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from example_interfaces.action import Fibonacci

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
        self._echo_service = self.create_service(Trigger, "matrix/echo", self._handle_echo)
        self._action_server = ActionServer(
            self,
            Fibonacci,
            "matrix/fibonacci",
            execute_callback=self._execute_fibonacci,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
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

    def _handle_echo(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        response.message = json.dumps(self._current_profile.as_dict())
        return response

    def _on_goal(self, goal_request: Fibonacci.Goal) -> GoalResponse:
        if goal_request.order <= 0:
            self.get_logger().warning("Rejecting fibonacci goal with non-positive order.")
            return GoalResponse.REJECT
        if goal_request.order > 30:
            self.get_logger().warning("Rejecting fibonacci goal with excessive order %d.", goal_request.order)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Fibonacci goal cancel requested.")
        return CancelResponse.ACCEPT

    def _execute_fibonacci(self, goal_handle) -> Fibonacci.Result:
        order = goal_handle.request.order
        sequence = [0, 1]
        feedback_msg = Fibonacci.Feedback()
        if hasattr(feedback_msg, "partial_sequence"):
            feedback_msg.partial_sequence = sequence.copy()
        elif hasattr(feedback_msg, "sequence"):
            feedback_msg.sequence = sequence.copy()

        if order == 1:
            sequence = [0]
        elif order == 2:
            sequence = [0, 1]
        else:
            for _ in range(2, order):
                sequence.append(sequence[-1] + sequence[-2])
                if hasattr(feedback_msg, "partial_sequence"):
                    feedback_msg.partial_sequence = sequence.copy()
                elif hasattr(feedback_msg, "sequence"):
                    feedback_msg.sequence = sequence.copy()
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.05)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence[:order]
        return result


def main() -> None:
    rclpy.init()
    responder = MatrixResponder()
    try:
        rclpy.spin(responder)
    except KeyboardInterrupt:
        responder.get_logger().info("Responder shutting down.")
    finally:
        responder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
