#!/usr/bin/env bash
# Phase A: minimal diff drive in Gazebo with stock controllers only.
set +u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG="$WS/case_study/logs"
mkdir -p "$LOG"
export DISPLAY=
# The sandbox makes ~/.ros and ~/.gazebo read-only; give Gazebo and ROS a writable HOME.
export HOME="$LOG/home"
mkdir -p "$HOME"
export ROS_LOG_DIR="$HOME/.ros/log"
export GAZEBO_LOG_PATH="$LOG"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash" 2>/dev/null || true

sed "s#__CONTROLLERS_YAML__#$WS/case_study/config/controllers_phaseA.yaml#" \
  "$WS/case_study/urdf/diff_drive.urdf" > "$LOG/diff_drive.generated.urdf"

PIDS=()
cleanup() { for p in ${PIDS[@]+"${PIDS[@]}"}; do kill -9 "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  > "$LOG/gzserver.log" 2>&1 &
PIDS+=($!)
sleep 6

ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat "$LOG/diff_drive.generated.urdf")" \
  -p use_sim_time:=true > "$LOG/rsp.log" 2>&1 &
PIDS+=($!)
sleep 3

ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity diff_drive \
  > "$LOG/spawn.log" 2>&1 &
PIDS+=($!)
sleep 10

echo "[runner] spawning controllers"
timeout -s KILL 60 ros2 run controller_manager spawner joint_state_broadcaster \
  > "$LOG/spawner_jsb.log" 2>&1
echo "[runner] joint_state_broadcaster exit=$?"
timeout -s KILL 60 ros2 run controller_manager spawner velocity_controller \
  > "$LOG/spawner_vel.log" 2>&1
echo "[runner] velocity_controller exit=$?"

echo "[runner] commanding both wheels at +2 rad/s for 6 s"
timeout -s KILL 8 ros2 topic pub -r 20 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [2.0, 2.0]}" > "$LOG/pub.log" 2>&1

python3 "$WS/case_study/scripts/measure_joint_states.py" 8
echo "[runner] done"
