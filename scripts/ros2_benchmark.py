#!/usr/bin/env python3
"""
ros2_benchmark.py
Author: Vishnuprasad Prachandabhanu
Description:
  Benchmarks ROS 2 pub-sub latency under different RMW and QoS configurations.
  Uses a simple ping-pong setup to measure round-trip latency.
"""

import argparse
import time
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class PingNode(Node):
    def __init__(self, qos_profile):
        super().__init__('ping_node')
        self.publisher = self.create_publisher(String, 'ping_topic', qos_profile)
        self.subscriber = self.create_subscription(String, 'pong_topic', self.callback, qos_profile)
        self.received_time = None
        self.sent_time = None
        self.latencies = []
        self.iterations = 100

    def callback(self, msg):
        self.received_time = time.time()
        latency = (self.received_time - self.sent_time) * 1000  # ms
        self.latencies.append(latency)
        if len(self.latencies) < self.iterations:
            self.send_ping()
        else:
            self.get_logger().info("Benchmark complete.")
            rclpy.shutdown()

    def send_ping(self):
        self.sent_time = time.time()
        msg = String()
        msg.data = f"ping-{len(self.latencies)}"
        self.publisher.publish(msg)


class PongNode(Node):
    def __init__(self, qos_profile):
        super().__init__('pong_node')
        self.subscriber = self.create_subscription(String, 'ping_topic', self.callback, qos_profile)
        self.publisher = self.create_publisher(String, 'pong_topic', qos_profile)

    def callback(self, msg):
        pong = String()
        pong.data = f"pong-{msg.data}"
        self.publisher.publish(pong)


def run_benchmark(distro, rmw, reliability, durability, output_file):
    # QoS setup
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if durability == "transient_local" else DurabilityPolicy.VOLATILE,
        depth=10
    )

    # Initialize two separate nodes
    rclpy.init()
    ping_node = PingNode(qos_profile)
    pong_node = PongNode(qos_profile)

    # Spin both nodes in parallel
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ping_node)
    executor.add_node(pong_node)

    # Start ping
    ping_node.send_ping()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ping_node.destroy_node()
        pong_node.destroy_node()
        rclpy.shutdown()

    # Save results
    if ping_node.latencies:
        df = pd.DataFrame({
            "iteration": range(len(ping_node.latencies)),
            "latency_ms": ping_node.latencies
        })
        df["mean_latency_ms"] = np.mean(ping_node.latencies)
        df["max_latency_ms"] = np.max(ping_node.latencies)
        df["rmw"] = rmw
        df["distro"] = distro
        df["reliability"] = reliability
        df["durability"] = durability
        df.to_csv(output_file, index=False)
        print(f"[✅] Saved results to {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ROS2 Benchmark Tool")
    parser.add_argument("--distro", required=True)
    parser.add_argument("--rmw", required=True)
    parser.add_argument("--reliability", choices=["reliable", "besteffort"], default="reliable")
    parser.add_argument("--durability", choices=["volatile", "transient_local"], default="volatile")
    parser.add_argument("--output", default="results.csv")
    args = parser.parse_args()

    run_benchmark(args.distro, args.rmw, args.reliability, args.durability, args.output)
