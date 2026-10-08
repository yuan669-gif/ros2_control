#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0.
"""Measure the per-level lag of the two-phase cascade ON GAZEBO PHYSICS.

Run while `two_phase_demo_gazebo.launch.py` is up, once per mode:

    python3 record_chain_lag.py <two_phase: true|false>

It subscribes to the three cascade controllers' `cycle_diagnostics` topics, waits until all three
report, then steps the chain ROOT's reference and reports, for every level, how many control cycles
after the leaf saw the step that level saw it.

Why this works on real hardware: each message carries `[cycle, estimate, child_published, stamp_s]`,
where the stamp is the manager's time for that cycle and is therefore SHARED by all three members.
The controllers are activated at different moments, so their own counters are not comparable -- the
stamp is what makes the series line up.

The physics cannot contaminate the number: with a `position` command interface the joint follows the
command within a physics step (1 ms), an order of magnitude below the 10 ms control period, and the
lag is counted in control cycles from the LEAF's own first movement, not in absolute time.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

LEVELS = ["tp_leaf", "tp_mid", "tp_root"]
STEP_VALUE = 0.75
THRESHOLD = 0.05


class Recorder(Node):
    def __init__(self, two_phase):
        super().__init__("chain_lag_recorder")
        self.series = {name: [] for name in LEVELS}
        self.publisher = self.create_publisher(Float64, "/tp_root/reference", 10)
        for name in LEVELS:
            self.create_subscription(
                Float64MultiArray,
                f"/{name}/cycle_diagnostics",
                lambda msg, name=name: self.series[name].append(tuple(msg.data)),
                50,
            )
        self.two_phase = two_phase

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        self.get_logger().info("waiting for the cascade to report ...")
        # Discovery on a loaded machine is not instant: give every level time to appear before
        # concluding that it is silent.
        deadline = time.time() + 45.0
        while time.time() < deadline:
            self.spin_for(1.0)
            missing = [name for name in LEVELS if not self.series[name]]
            if not missing:
                break
            self.get_logger().info(f"still waiting for: {missing}")
        for name in LEVELS:
            if not self.series[name]:
                self.get_logger().error(f"no diagnostics from {name}; is the demo running?")
                return 2
        # WAIT FOR A SETTLED BASELINE. Right after the stack comes up the joint is still moving
        # towards whatever position it was left in, and a baseline taken during that transient is
        # meaningless: the "first change" then fires near convergence and every level appears to move
        # on the same cycle. Measured exactly that way before this wait existed.
        self.get_logger().info("waiting for the leaf to settle before stepping ...")
        settle_deadline = time.time() + 40.0
        while time.time() < settle_deadline:
            self.spin_for(1.0)
            recent = [sample[1] for sample in self.series["tp_leaf"][-100:]]
            if len(recent) >= 100 and (max(recent) - min(recent)) < 0.01:
                break
        self.get_logger().info("all three levels are reporting; stepping the root reference")

        self.spin_for(1.0)
        step_index = {name: len(self.series[name]) for name in LEVELS}
        start = time.time()
        while time.time() - start < 2.0:
            self.publisher.publish(Float64(data=STEP_VALUE))
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_for(2.0)

        # The first cycle, per level, in which the estimate leaves its pre-step baseline.
        first_change = {}
        for name in LEVELS:
            pre = [sample[1] for sample in self.series[name][: step_index[name]]]
            baseline = sum(pre) / len(pre) if pre else 0.0
            for sample in self.series[name][step_index[name] :]:
                if abs(sample[1] - baseline) > THRESHOLD:
                    first_change[name] = (sample[3], sample[0], sample[1])
                    break

        if "tp_leaf" not in first_change:
            self.get_logger().error("the leaf never moved: the hardware did not follow the command")
            return 3
        missing = [name for name in LEVELS if name not in first_change]
        if missing:
            self.get_logger().error(f"levels that never responded: {missing}")
            return 4

        leaf_stamp = first_change["tp_leaf"][0]
        period = None
        stamps = [sample[3] for sample in self.series["tp_leaf"][-20:]]
        if len(stamps) > 1:
            period = (stamps[-1] - stamps[0]) / (len(stamps) - 1)

        print(f"\n=== Gazebo cascade, two_phase_execution = {self.two_phase} ===")
        print(f"control period from the stamps: {period * 1e3:.2f} ms" if period else "")
        print(f"{'level':10s} {'estimate at first change':>24s} {'lag (cycles)':>13s}")
        for name in LEVELS:
            stamp, _cycle, estimate = first_change[name]
            lag = 0 if period in (None, 0) else round((stamp - leaf_stamp) / period)
            print(f"{name:10s} {estimate:24.4f} {lag:13d}")
        print()
        return 0


def main():
    two_phase = (sys.argv[1].lower() == "true") if len(sys.argv) > 1 else True
    rclpy.init()
    node = Recorder(two_phase)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
