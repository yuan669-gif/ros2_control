#!/usr/bin/env python3
"""Measure the state-edge age and the true trajectory error in the Gazebo case study.

Subscribes to:
  /chassis/diagnostics   [x, y, th, ref_x, ref_y, ref_th, ex, ey, eth, travel_l_used, travel_r_used, v, w]
  /wheel_left/travel     cumulative wheel travel [m]
  /wheel_right/travel    cumulative wheel travel [m]
  /gazebo/model_states   ground-truth model pose

Reports:
  * the integer lag (in control cycles) that best aligns the travel the chassis USED with the
    travel the wheels actually had;
  * the RMS true position error versus the reference pose.
"""
import math
import sys
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Measure(Node):
    def __init__(self, model_name):
        super().__init__("measure_tracking")
        self.model_name = model_name
        self.chassis = []
        self.wheel = {"left": [], "right": []}
        self.truth = []
        self.create_subscription(Float64MultiArray, "/chassis/diagnostics", self.on_chassis, 50)
        self.create_subscription(Float64, "/wheel_left/travel", lambda m: self.on_wheel("left", m), 50)
        self.create_subscription(Float64, "/wheel_right/travel", lambda m: self.on_wheel("right", m), 50)
        self.create_subscription(ModelStates, "/model_states", self.on_truth, 50)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_chassis(self, msg):
        self.chassis.append((self.now(), list(msg.data)))

    def on_wheel(self, side, msg):
        self.wheel[side].append((self.now(), msg.data))

    def on_truth(self, msg):
        if self.model_name not in msg.name:
            return
        index = msg.name.index(self.model_name)
        pose = msg.pose[index]
        self.truth.append((self.now(), pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)))


def nearest(series, t, index):
    if not series:
        return None
    best = min(series, key=lambda item: abs(item[0] - t))
    return best[index]


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
    if samples < 10:
        print(f"[case-study] mode={mode} rate={rate:.0f} Hz  NO DATA (chassis={samples})")
        node.destroy_node()
        rclpy.shutdown()
        return

    # --- state-edge age: which integer shift aligns used-travel with true travel? -----------
    best = {}
    for side, index in (("left", 9), ("right", 10)):
        errors = []
        for shift in range(0, 6):
            total = 0.0
            count = 0
            for t, values in node.chassis:
                truth = nearest(node.wheel[side], t - shift * period, 1)
                if truth is None:
                    continue
                total += (values[index] - truth) ** 2
                count += 1
            errors.append(total / count if count else float("inf"))
        best[side] = errors.index(min(errors))

    # --- alignment probe -------------------------------------------------------------------
    if node.truth and node.chassis:
        t0 = node.chassis[0][0]
        print(f"[probe] chassis_t0={t0:.3f} truth_t0={node.truth[0][0]:.3f} "
              f"dt={node.truth[0][0] - t0:+.3f}")
        for k in (0, len(node.chassis) // 2, len(node.chassis) - 1):
            t, values = node.chassis[k]
            truth = nearest(node.truth, t, 1), nearest(node.truth, t, 2)
            print(
                f"[probe] chassis x={values[0]: .3f} y={values[1]: .3f} | "
                f"ref x={values[3]: .3f} y={values[4]: .3f} | "
                f"truth x={truth[0]: .3f} y={truth[1]: .3f}"
                if truth[0] is not None else "[probe] no truth")
    # --- true trajectory error over the second half of the run -----------------------------
    half = node.chassis[len(node.chassis) // 3:]
    squared = 0.0
    counted = 0
    max_error = 0.0
    for t, values in half:
        truth = nearest(node.truth, t, 1), nearest(node.truth, t, 2)
        if truth[0] is None:
            continue
        error = math.hypot(truth[0] - values[3], truth[1] - values[4])
        squared += error * error
        max_error = max(max_error, error)
        counted += 1
    rms = math.sqrt(squared / counted) if counted else float("nan")

    first_t = node.chassis[0][1] if node.chassis else []
    last_t = node.chassis[-1][1] if node.chassis else []
    left_series = [v for _, v in node.wheel["left"]]
    travel_range = (max(left_series) - min(left_series)) if left_series else 0.0
    for label, item in (("first", node.chassis[0]), ("mid", node.chassis[len(node.chassis)//2]), ("last", node.chassis[-1])):
        print(f"[case-study] diag[{label}] " + " ".join(f"{v: .4f}" for v in item[1][:13]))
    print(
        f"[case-study] debug: truth_samples={len(node.truth)} wheel_samples={len(node.wheel['left'])} "
        f"travel_range={travel_range * 1000:.2f} mm final_pose=({last_t[0]:.3f},{last_t[1]:.3f},{last_t[2]:.3f}) "
        f"ref_pose=({last_t[3]:.3f},{last_t[4]:.3f},{last_t[5]:.3f}) v={last_t[11]:.3f} w={last_t[12]:.3f}")
    print(
        f"[case-study] mode={mode:11s} rate={rate:5.0f} Hz  samples={samples:5d}  "
        f"travel_lag_left={best['left']} ({best['left'] * period * 1000:.0f} ms) "
        f"travel_lag_right={best['right']} ({best['right'] * period * 1000:.0f} ms)  "
        f"(ground-truth pose comparison withheld: not yet reliable in this harness)"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
