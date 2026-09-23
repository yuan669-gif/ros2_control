#!/usr/bin/env python3
"""Measure wheel motion from /joint_states for a fixed duration."""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Measure(Node):
    def __init__(self):
        super().__init__("measure_joint_states")
        self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.first_position = {}
        self.last_position = {}
        self.last_velocity = {}
        self.samples = 0

    def cb(self, msg):
        self.samples += 1
        for index, name in enumerate(msg.name):
            if name not in self.first_position:
                self.first_position[name] = msg.position[index]
            self.last_position[name] = msg.position[index]
            if index < len(msg.velocity):
                self.last_velocity[name] = msg.velocity[index]


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = Measure()
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"[measure] samples={node.samples} over {duration:.1f}s")
    for name in sorted(node.first_position):
        delta = node.last_position[name] - node.first_position[name]
        velocity = node.last_velocity.get(name, float("nan"))
        print(
            f"[measure] {name:20s} position_delta={delta:+.4f} rad "
            f"final_velocity={velocity:+.4f} rad/s"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
