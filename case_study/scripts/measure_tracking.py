#!/usr/bin/env python3
"""Measure the state-edge lag and the true trajectory error in the Gazebo case study.

Lag is measured from CONTROL CYCLE NUMBERS, not from timestamps
----------------------------------------------------------------
An earlier revision aligned the chassis's used-travel against the wheels' published travel with a
subscriber-side nearest-neighbour search on arrival times. That mixes in DDS queueing, per-topic
offsets, packet loss and the difference between simulation and wall clocks, and it could also
report a spurious lag of 0 when the wheels produced no data at all (every error came out inf and
`errors.index(min(errors))` returned 0). See doc/REVIEW_HUMBLE_WORK_2026-09-23.md R9.

The controllers now stamp cycle numbers, so the lag is an exact integer difference:

    lag = chassis_cycle - wheel_cycle_of_the_value_the_chassis_read

Each controller increments its own counter once per control cycle, so this is a scheduling lag by
construction and needs no clock alignment. Missing data is a FAILURE, not a zero.

Subscribes to:
  /chassis/diagnostics   [x, y, th, ref_x, ref_y, ref_th, ex, ey, eth,
                          used_left, used_right, v, w,
                          chassis_cycle, wheel_left_cycle, wheel_right_cycle]
  /wheel_left/travel     cumulative wheel travel [m]
  /wheel_right/travel    cumulative wheel travel [m]
  /model_states          Gazebo ground-truth model pose

Usage: measure_tracking.py <duration_s> <rate_hz> [model_name] [mode_label]
"""
import math
import statistics
import sys
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

# Index layout of /chassis/diagnostics, in one place so a format change is a single edit.
DIAG_X = 0
DIAG_Y = 1
DIAG_TH = 2
DIAG_REF_X = 3
DIAG_REF_Y = 4
DIAG_REF_TH = 5
DIAG_USED_LEFT = 9
DIAG_USED_RIGHT = 10
DIAG_V = 11
DIAG_W = 12
DIAG_CYCLE = 13
DIAG_LEFT_CYCLE = 14
DIAG_RIGHT_CYCLE = 15
DIAG_MIN_LEN = 16


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Measure(Node):
    def __init__(self, model_name):
        super().__init__("measure_tracking")
        self.model_name = model_name
        self.chassis = []
        self.short_rows = 0
        self.wheel = {"left": [], "right": []}
        self.truth = []
        self.create_subscription(Float64MultiArray, "/chassis/diagnostics", self.on_chassis, 50)
        self.create_subscription(
            Float64, "/wheel_left/travel", lambda m: self.on_wheel("left", m), 50
        )
        self.create_subscription(
            Float64, "/wheel_right/travel", lambda m: self.on_wheel("right", m), 50
        )
        self.create_subscription(ModelStates, "/model_states", self.on_truth, 50)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_chassis(self, msg):
        if len(msg.data) < DIAG_MIN_LEN:
            self.short_rows += 1
            return
        self.chassis.append((self.now(), list(msg.data)))

    def on_wheel(self, side, msg):
        self.wheel[side].append((self.now(), msg.data))

    def on_truth(self, msg):
        if self.model_name not in msg.name:
            return
        index = msg.name.index(self.model_name)
        pose = msg.pose[index]
        self.truth.append(
            (self.now(), pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        )


def nearest(series, t, index):
    if not series:
        return None
    best = min(series, key=lambda item: abs(item[0] - t))
    return best[index]


def fail(message):
    print(f"[case-study] FAILED: {message}")
    return 1


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    model = sys.argv[3] if len(sys.argv) > 3 else "diff_drive"
    mode = sys.argv[4] if len(sys.argv) > 4 else "?"

    rclpy.init()
    node = Measure(model)
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    period = 1.0 / rate
    samples = len(node.chassis)
    short_rows = node.short_rows

    if samples < 10:
        node.destroy_node()
        rclpy.shutdown()
        return fail(f"no usable chassis diagnostics (samples={samples}, short_rows={short_rows})")

    # Missing wheel data must abort, not be reported as zero lag.
    for side in ("left", "right"):
        if not node.wheel[side]:
            node.destroy_node()
            rclpy.shutdown()
            return fail(f"no /wheel_{side}/travel samples; lag cannot be measured")

    if short_rows:
        print(
            f"[case-study] note: ignored {short_rows} diagnostics with fewer than {DIAG_MIN_LEN} "
            "fields (stale controller build?)"
        )

    # --- state-edge lag, from cycle numbers -------------------------------------------------
    lag = {"left": [], "right": []}
    for _, values in node.chassis:
        chassis_cycle = int(values[DIAG_CYCLE])
        for side, key in (("left", DIAG_LEFT_CYCLE), ("right", DIAG_RIGHT_CYCLE)):
            wheel_cycle = int(values[key])
            if wheel_cycle > 0:
                lag[side].append(chassis_cycle - wheel_cycle)

    for side in ("left", "right"):
        if not lag[side]:
            node.destroy_node()
            rclpy.shutdown()
            return fail(f"no usable cycle stamps for wheel_{side}")

    summary = {}
    for side in ("left", "right"):
        values = lag[side]
        summary[side] = (int(statistics.median(values)), min(values), max(values), len(values))

    # --- provisional true-trajectory error --------------------------------------------------
    truth_seen = len(node.truth)
    rms = float("nan")
    if truth_seen:
        half = node.chassis[len(node.chassis) // 3:]
        squared = 0.0
        counted = 0
        for t, values in half:
            tx = nearest(node.truth, t, 1)
            ty = nearest(node.truth, t, 2)
            if tx is None:
                continue
            squared += (tx - values[DIAG_REF_X]) ** 2 + (ty - values[DIAG_REF_Y]) ** 2
            counted += 1
        if counted:
            rms = math.sqrt(squared / counted)

    last = node.chassis[-1][1]
    left_series = [v for _, v in node.wheel["left"]]
    travel_range = (max(left_series) - min(left_series)) if left_series else 0.0

    print(
        f"[case-study] mode={mode:11s} rate={rate:5.0f} Hz  samples={samples:5d}  "
        f"chassis_cycles={int(last[DIAG_CYCLE]):5d}  "
        f"final_pose=({last[DIAG_X]:.3f},{last[DIAG_Y]:.3f}) "
        f"ref=({last[DIAG_REF_X]:.3f},{last[DIAG_REF_Y]:.3f})  "
        f"travel_range={travel_range * 1000:.1f} mm"
    )
    for side in ("left", "right"):
        median, lo, hi, n = summary[side]
        print(
            f"[case-study] lag_{side:5s} = {median} cycles (min {lo}, max {hi}, n={n})  "
            f"= {median * period * 1000:.1f} ms at the CONFIGURED {rate:.0f} Hz period"
        )
    print(
        "[case-study] NOTE: the cycle figure is an exact integer scheduling lag. The millisecond "
        "figure is DERIVED from the configured control period, not measured on a wall clock."
    )
    if truth_seen:
        print(
            f"[case-study] provisional RMS true-position error vs reference = {rms:.3f} m "
            f"(truth samples={truth_seen}); per GAZEBO_CASE_STUDY.md section 4 this metric is not "
            "trustworthy yet and no conclusion may rest on it"
        )
    else:
        print("[case-study] no /model_states samples; true-trajectory error not computed")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
