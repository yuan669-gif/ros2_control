#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0.
"""Drill the chain root's reference so the arm keeps moving and can be watched.

    python3 drill_reference.py [--amplitude 0.8] [--period 6.0] [--topic /tp_root/reference]

A square wave, because a single step (what `record_chain_lag.py` publishes) moves the arm once and then
it sits still -- fine for measuring a lag, useless for looking at. This is a *visualisation* aid: do not
run it while measuring, since a reference that keeps changing moves the baseline the recorder relies on.

The wave reaches the hardware through the whole cascade, so what you see in Gazebo is the chain actually
working: the leaf's position tracking the root's reference, with the intermediate levels' estimates in
between.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/tp_root/reference")
    parser.add_argument("--amplitude", type=float, default=0.8, help="radians (revolute joint)")
    parser.add_argument("--period", type=float, default=6.0, help="seconds for a full +/- cycle")
    parser.add_argument("--rate", type=float, default=50.0, help="publish rate in Hz")
    args = parser.parse_args()

    rclpy.init()
    node = Node("drill_reference")
    publisher = node.create_publisher(Float64, args.topic, 10)
    node.get_logger().info(
        f"drilling {args.topic}: +/-{args.amplitude} rad every {args.period:.1f} s "
        f"at {args.rate:.0f} Hz (Ctrl-C to stop)"
    )

    start = time.monotonic()
    try:
        while rclpy.ok():
            phase = ((time.monotonic() - start) % args.period) / args.period
            publisher.publish(Float64(data=args.amplitude if phase < 0.5 else -args.amplitude))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
