#!/usr/bin/env bash
# Run the case-study A/B (single-pass vs two-pass) with whole-run retries.
#
# Why retries: on this machine `gzserver` aborts on roughly 1 startup/shutdown in 3
# (`free(): invalid pointer`, `corrupted size vs. prev_size`, or a segfault), which kills the run
# before the measurement. Every attempt uses a fresh Gazebo master port (phase_b.sh picks one), so a
# retry never reuses a half-dead server.
#
# Usage: retry_phase_b.sh [rate_hz] [duration_s] [max_attempts_per_mode]
set +u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RATE="${1:-50}"
DURATION="${2:-20}"
MAX_ATTEMPTS="${3:-4}"

for MODE in false true; do
  if [ "$MODE" = "true" ]; then LABEL="two-pass"; else LABEL="single-pass"; fi
  ok=0
  for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    echo "=== [$LABEL] attempt $attempt/$MAX_ATTEMPTS ==="
    out=$(timeout -s KILL 600 bash "$WS/case_study/scripts/phase_b.sh" "$RATE" "$MODE" "$DURATION" 2>&1)
    echo "$out" | grep -E "case-study\]|runner.*(exit|done)|Aborted|Segmentation|core dumped" | tail -25
    if echo "$out" | grep -q "lag_left"; then ok=1; break; fi
    echo "--- [$LABEL] attempt $attempt produced no measurement; retrying ---"
    # Kill leftovers of the failed attempt. Patterns are given as exact names where possible so this
    # script can never match its own command line.
    pkill -9 -x gzserver 2>/dev/null
    pkill -9 -x robot_state_publisher 2>/dev/null
    pkill -9 -f 'spawn_entity\.py' 2>/dev/null
    sleep 5
  done
  if [ "$ok" = "1" ]; then echo "=== [$LABEL] MEASURED ==="; else echo "=== [$LABEL] ALL ATTEMPTS FAILED ==="; fi
done
echo "=== RUNNER DONE ==="
