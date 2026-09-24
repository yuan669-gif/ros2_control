#!/usr/bin/env bash
# ThreadSanitizer check of the membership publish protocol (review R8).
#
#   racy   : the reviewed pattern (plain vector written by the idle thread, read by the control
#            thread) MUST be reported. This is the control that proves the check is not vacuous.
#   atomic : the current pattern (immutable snapshot, atomic_store / atomic_load) MUST be clean.
#
# TSan needs ASLR disabled on this machine: with the default mmap_rnd_bits the runtime aborts with
# "unexpected memory mapping", so every TSan binary is launched through `setarch -R`.
set +u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$WS/log/tsan"
mkdir -p "$OUT"
BIN="$OUT/tsan_publish_protocol"

echo "[tsan] compiling with -fsanitize=thread"
g++ -std=c++17 -O1 -g -fsanitize=thread -fno-omit-frame-pointer \
  -pthread "$WS/hierarchical_control/test/tsan_publish_protocol.cpp" -o "$BIN" || exit 2

run_mode() {
  local mode="$1"
  local log="$OUT/$mode.log"
  setarch "$(uname -m)" -R "$BIN" "--mode=$mode" > "$log" 2>&1
  echo "$?"
}

echo "[tsan] running --mode=racy (a report is REQUIRED)"
status_racy=$(run_mode racy)
if grep -q "WARNING: ThreadSanitizer: data race" "$OUT/racy.log"; then
  echo "[tsan] racy   : data race reported (expected) -- the harness detects the reviewed pattern"
  racy_ok=1
else
  echo "[tsan] racy   : NO race reported -- the harness is vacuous, treating this as a FAILURE"
  racy_ok=0
fi
grep -m1 -A3 "WARNING: ThreadSanitizer" "$OUT/racy.log" | sed 's/^/[tsan]   /'

echo "[tsan] running --mode=atomic (a clean run is REQUIRED)"
status_atomic=$(run_mode atomic)
if grep -q "WARNING: ThreadSanitizer" "$OUT/atomic.log"; then
  echo "[tsan] atomic : reported a race -- the fix does not hold"
  atomic_ok=0
else
  echo "[tsan] atomic : clean (exit $status_atomic)"
  atomic_ok=1
fi
tail -2 "$OUT/atomic.log" | sed 's/^/[tsan]   /'

if [ "$racy_ok" = "1" ] && [ "$atomic_ok" = "1" ]; then
  echo "[tsan] RESULT: PASS (racy reported, atomic clean)"
  exit 0
fi
echo "[tsan] RESULT: FAIL (racy_ok=$racy_ok atomic_ok=$atomic_ok)"
exit 1
