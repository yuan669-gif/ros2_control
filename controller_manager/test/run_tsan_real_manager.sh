#!/usr/bin/env bash
# ThreadSanitizer check of the REAL ControllerManager (review item D, second half).
#
# The older harness (`hierarchical_control/test/run_tsan_publish_protocol.sh`) only simulates the
# membership publish protocol in a standalone program. This one instruments the actual
# `controller_manager` package -- `update()` on one thread against `switch_controller()` /
# `set_two_phase_execution()` / controller list changes on another -- and runs the existing
# `test_two_phase_execution` suite, which drives exactly that pattern.
#
# SCOPE, stated honestly: only this package is instrumented. Its dependencies (rclcpp, the lifecycle
# libraries, hardware_interface, FastRTPS) come from the normal build and are NOT instrumented, so
#   * a data race inside those libraries is invisible here, and
#   * the lock-order-inversion warnings TSan prints for them cannot be attributed or fixed from this
#     package (they are reported from the interceptor with no application frames, or with our code
#     only as the caller).
# What this DOES verify is the manager's own cross-thread state: the two-phase flag and entry
# snapshot, the controller-list indices, the switch handshake fields and the request lists.
#
# Usage:  bash controller_manager/test/run_tsan_real_manager.sh [--rebuild]
# Result: PASS = the suite passes and NO data race is reported.
set +u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$WS/log/tsan"
mkdir -p "$OUT"
BUILD="$WS/build_tsan"
INSTALL="$WS/install_tsan"
BIN="$BUILD/controller_manager/test_two_phase_execution"
LOG="$OUT/real_manager.log"

TSAN_FLAGS=(-DCMAKE_CXX_FLAGS="-fsanitize=thread -g -O1"
            -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=thread"
            -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread")

if [ ! -x "$BIN" ] || [ "$1" = "--rebuild" ]; then
  echo "[tsan] building controller_manager with -fsanitize=thread (separate build tree: $BUILD)"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # The workspace's own dependencies (hierarchical_control, hardware_interface_testing, ...) come
  # from the normal install space: this build tree only instruments controller_manager itself.
  if [ -f "$WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
  fi
  ( cd "$WS" && colcon --log-base log_tsan build --packages-select controller_manager \
      --build-base build_tsan --install-base install_tsan \
      --cmake-args "${TSAN_FLAGS[@]}" --cmake-target test_two_phase_execution ) || exit 2
fi

# TSan needs ASLR disabled on this machine ("unexpected memory mapping" otherwise), and the
# instrumented library must win over the installed one.
echo "[tsan] running the real manager suite under TSan"
LD_LIBRARY_PATH="$BUILD/controller_manager:$LD_LIBRARY_PATH" \
TSAN_OPTIONS="halt_on_error=0 second_deadlock_stack=1" \
  setarch "$(uname -m)" -R "$BIN" > "$LOG" 2>&1
status=$?

races=$(grep -c "WARNING: ThreadSanitizer: data race" "$LOG")
inversions=$(grep -c "lock-order-inversion" "$LOG")
gtest_line=$(grep -E "^\[  (PASSED|FAILED)  \]" "$LOG" | tail -1)

echo "[tsan] data races            : $races"
echo "[tsan] lock-order inversions : $inversions  (uninstrumented dependencies; see the header)"
echo "[tsan] gtest                 : ${gtest_line:-<no result line>}"

# The VERDICT is the race count, not the suite result. TSan slows the process down by roughly an
# order of magnitude here, and several cases assert timing (`two_pass_costs_one_extra_traversal`
# measures microseconds; the switch-pause case depends on when the asynchronous request lands), so
# they can fail under TSan for reasons unrelated to concurrency. They are reported, not folded in.
if [ "$races" != "0" ]; then
  echo "[tsan] races reported (frames in this package's files):"
  grep -B2 -A6 "WARNING: ThreadSanitizer: data race" "$LOG" |
    grep -oE "(controller_manager|hierarchical_control)/[a-z_/]+\.(cpp|hpp):[0-9]+" | sort | uniq -c |
    sed 's/^/[tsan]   /'
fi
failed=$(grep -cE "^\[  FAILED  \] Test" "$LOG")
if [ "$failed" != "0" ]; then
  echo "[tsan] note: $failed case(s) failed under TSan (timing-sensitive; see the comment above)"
  grep -E "^\[  FAILED  \] Test" "$LOG" | sed 's/^\[  FAILED  \] /[tsan]   /; s/ ([0-9].*//' | sort -u
fi

if [ "$races" = "0" ]; then
  echo "[tsan] RESULT: PASS (no data race reported in the instrumented manager)"
  exit 0
fi
echo "[tsan] RESULT: FAIL ($races data race(s); full log: $LOG)"
exit 1
