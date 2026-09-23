#!/usr/bin/env bash
# Phase B: single-pass vs two-pass execution of the same controllers in Gazebo.
# usage: phase_b.sh <update_rate> <two_phase:true|false> <duration>
set +u
RATE="${1:-50}"
TWO_PHASE="${2:-false}"
DURATION="${3:-20}"
if [ "$TWO_PHASE" = "true" ]; then LEGACY="false"; MODE="two-pass"; else LEGACY="true"; MODE="single-pass"; fi

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG="$WS/case_study/logs"
RUN="$LOG/run_${RATE}hz_${MODE}"
mkdir -p "$RUN"
export DISPLAY=
export HOME="$LOG/home"
mkdir -p "$HOME"
export ROS_LOG_DIR="$HOME/.ros/log"
export GAZEBO_LOG_PATH="$RUN"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash" 2>/dev/null || true

sed -e "s#__RATE__#$RATE#" -e "s#__TWO_PHASE__#$TWO_PHASE#" -e "s#__LEGACY__#$LEGACY#" \
  "$WS/case_study/config/controllers_phaseB.yaml.in" > "$RUN/controllers.yaml"
sed -e "s#__CONTROLLERS_YAML__#$RUN/controllers.yaml#" \
  "$WS/case_study/urdf/diff_drive.urdf" > "$RUN/diff_drive.generated.urdf"

# A fresh master port per run: leftover Gazebo processes must never be reused across runs.
PORT=$((11400 + RANDOM % 400))
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
echo "[runner] master $GAZEBO_MASTER_URI"

PIDS=()
cleanup() {
  # SIGTERM first so Gazebo flushes its log, then force-kill whatever survives.
  for p in ${PIDS[@]+"${PIDS[@]}"}; do kill -TERM "$p" 2>/dev/null || true; done
  sleep 2
  for p in ${PIDS[@]+"${PIDS[@]}"}; do kill -9 "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

gzserver --verbose "$WS/case_study/world/case_study.world" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > "$RUN/gzserver.log" 2>&1 &
PIDS+=($!)
sleep 6

ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat "$RUN/diff_drive.generated.urdf")" -p use_sim_time:=true \
  > "$RUN/rsp.log" 2>&1 &
PIDS+=($!)
sleep 3

# Spawn clear of the ground: the wheels' lowest point is 0.15 m below the model origin.
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity diff_drive -z 0.16 \
  > "$RUN/spawn.log" 2>&1 &
PIDS+=($!)
sleep 10

for controller in joint_state_broadcaster wheel_left wheel_right chassis; do
  timeout -s KILL 60 ros2 run controller_manager spawner "$controller" \
    > "$RUN/spawner_$controller.log" 2>&1
  echo "[runner:$MODE@$RATE] spawn $controller exit=$?"
done

echo "[runner:$MODE@$RATE] controller order:"
timeout -s KILL 20 ros2 control list_controllers 2>/dev/null | head -8

echo "[runner:$MODE@$RATE] measuring for ${DURATION}s"
python3 "$WS/case_study/scripts/measure_tracking.py" "$DURATION" "$RATE" diff_drive "$MODE"
echo "[runner:$MODE@$RATE] done"
