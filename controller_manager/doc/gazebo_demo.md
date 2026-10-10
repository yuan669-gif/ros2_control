# Running the Gazebo demo (real physics, real DDS)

This is the operating procedure for the Gazebo variant of the two-phase demo: the cascade runs on
`gazebo_ros2_control/GazeboSystem` hardware, the controllers are loaded by the real `spawner` over DDS,
and a recorder measures how many control cycles late each level sees a step.

Every command below was run on this machine; the expected outputs are copied from a real run, not
written from intent.

---

## 0. Prerequisites and build

```bash
# ROS 2 Humble, plus gazebo_ros2_control, robot_state_publisher and gazebo_ros
ls /opt/ros/humble/lib/libgazebo_ros2_control.so        # must exist
ls /opt/ros/humble/lib/gazebo_ros/spawn_entity.py       # must exist
ls /opt/ros/humble/lib/robot_state_publisher/robot_state_publisher

cd <workspace>            # the checkout of feature/two-phase-manager
source /opt/ros/humble/setup.bash
colcon build --packages-select controller_interface controller_manager
source install/setup.bash
```

**Environment quirks of this machine** (skip if not applicable to yours):

```bash
export ROS_LOG_DIR=$PWD/roslog     # ~/.ros/log is read-only in the container we use
export HOME=$PWD/gzhome            # gzserver wants to write ~/.gazebo; give it a writable HOME
mkdir -p "$ROS_LOG_DIR" "$HOME"
```

---

## 1. First: the deterministic evidence (no Gazebo, ~2 minutes)

Do this first. It is exact, has no physics or discovery in it, and if it fails the problem is the build,
not the simulator.

```bash
./build/controller_manager/test_two_phase_execution    # 31 tests
./build/controller_manager/test_two_phase_evidence     # 4 tests
```

Expected endings: `[  PASSED  ] 31 tests.` and `[  PASSED  ] 4 tests.`
The second binary prints one measured line, e.g.
`[evidence] allocations/cycle: single-pass 14, two-phase 8; us/cycle: single-pass 8, two-phase 3`.

---

## 2. Then the in-memory demo (~20 s): does the spawner/YAML path work?

```bash
timeout 45 ros2 launch controller_manager two_phase_demo.launch.py
```

Expected (the two lines that matter):

```
[controller_manager]: Two-phase execution requested by parameter: enabled (lag budget 0 cycle(s))
[spawner_tp_leaf]: Configured and activated tp_leaf
[spawner_tp_mid]:  Configured and activated tp_mid
[spawner_tp_root]: Configured and activated tp_root
```

If this fails, Gazebo will not work either: it exercises the same controller loading and the same YAML.

---

## 3. The Gazebo run

Two terminals (or run the launcher in the background, as below).

### 3.1 Terminal 1 — the system

```bash
export ROS_LOG_DIR=$PWD/roslog HOME=$PWD/gzhome
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch controller_manager two_phase_demo_gazebo.launch.py
```

Wait for this line, which is the ready signal:

```
[spawner_tp_root]: Configured and activated tp_root
```

Success also looks like:

```
[resource_manager]: Successful 'activate' of hardware 'GazeboTwoPhaseDemo'
[controller_manager]: Two-phase execution requested by parameter: enabled (lag budget 0 cycle(s))
```

### 3.2 Terminal 2 — the measurement

```bash
export ROS_LOG_DIR=$PWD/roslog HOME=$PWD/gzhome
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 install/controller_manager/share/controller_manager/two_phase_demo/record_chain_lag.py true
```

The recorder waits for all three levels to report, waits for the joint to **settle**, then publishes the
step on the chain root's reference topic and prints:

```
=== Gazebo cascade, two_phase_execution = True ===
control period from the stamps: 10.00 ms
level      estimate at first change  lag (cycles)
tp_leaf                      0.7450             0
tp_mid                       0.3742             0
tp_root                      0.1879             0
```

---

## 3.3 Watching it: the GUI and something for the arm to do

`gzclient` is started by default **when a display exists**, and skipped when there is none (CI,
containers, `ssh` without `-X`), so the same launch works everywhere:

```bash
ros2 launch controller_manager two_phase_demo_gazebo.launch.py            # gui auto: on with DISPLAY
ros2 launch controller_manager two_phase_demo_gazebo.launch.py gui:=true  # force it on
ros2 launch controller_manager two_phase_demo_gazebo.launch.py gui:=false # force it off (MEASUREMENTS)
```

Check the default for your shell with
`ros2 launch controller_manager two_phase_demo_gazebo.launch.py --show-args`.

The arm will sit still, because nothing is commanding it. To watch the chain actually work, drill the
root's reference (a second terminal):

```bash
python3 install/controller_manager/share/controller_manager/two_phase_demo/drill_reference.py --period 6 --amplitude 0.8
```

The arm then swings back and forth through the whole cascade. Measured on this machine, with the GUI up:
the leaf's position oscillated between **-1.333 and +1.333 rad** for a +/-0.8 rad reference over a 25 s
window — visible motion, and the overshoot beyond the reference is the cascade's proportional character,
not a bug (this demo is not a tuned controller).

**Do not run the drill and `record_chain_lag.py` at the same time**: the recorder measures from a settled
baseline to a single step, and a reference that keeps moving destroys that.

If the window appears but stays black, force software rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch controller_manager two_phase_demo_gazebo.launch.py gui:=true
```

`dconf-CRITICAL ... Read-only file system` lines from `gzclient` are harmless here: they come from the
redirected `HOME` (§0), not from the simulation.

---

## 4. The comparison run (single pass)

The mode is a parameter, so it is decided before the controller manager starts:

* **Edit the YAML and relaunch.** Edit
  `install/controller_manager/share/controller_manager/two_phase_demo/two_phase_demo_gazebo_controllers.yaml`,
  set `two_phase_execution: false`, stop and restart Terminal 1, then run the recorder with `false`.
  Note the launch reads the **installed** copy; if you edit the source copy under
  `controller_manager/test/two_phase_demo/`, rebuild with
  `colcon build --packages-select controller_manager` first, or your edit will be overwritten.
* **Or change it live** (no relaunch):

  ```bash
  ros2 param set /controller_manager two_phase_execution false
  ```

  then run the recorder with `false`. This works, but on a loaded machine `ros2 param` sometimes fails to
  find the node (`Node not found`) because of slow discovery — retry, or use the relaunch route.

Expected for single pass:

```
=== Gazebo cascade, two_phase_execution = False ===
control period from the stamps: 10.00 ms
level      estimate at first change  lag (cycles)
tp_leaf                      0.7427             0
tp_mid                       0.3712             1
tp_root                      0.1864             2
```

That contrast — `0,0,0` against `0,1,2` — is the whole point of the demo: state that reaches the root in
the same cycle with two passes, and one cycle per level late without them.

---

## 5. gzserver crashes about one launch in three: retry

`gzserver` aborts during controller loading with

```
[gzserver-1] free(): invalid pointer
[ERROR] [gzserver-1]: process has died [pid ..., exit code -6, ...]
```

This is intermittent and unrelated to the mode (it happened in both). Retry; a clean run follows within a
few attempts. This snippet does the retry loop and stops at the first good run:

```bash
for attempt in 1 2 3 4 5; do
  echo "===== attempt $attempt ====="
  pkill -x gzserver; sleep 6; rm -rf roslog gzhome; mkdir -p roslog gzhome
  timeout 200 ros2 launch controller_manager two_phase_demo_gazebo.launch.py > gz.log 2>&1 &
  for i in $(seq 1 22); do
    grep -q "Configured and activated.*tp_root" gz.log 2>/dev/null && break; sleep 5
  done
  if grep -q "Configured and activated.*tp_root" gz.log && [ "$(pgrep -xc gzserver)" != "0" ]; then
    sleep 5
    python3 install/controller_manager/share/controller_manager/two_phase_demo/record_chain_lag.py true
    pkill -x gzserver; break
  fi
  echo "  crashed on launch, retrying"; pkill -x gzserver
done
```

**Also note**: `ros2 launch` keeps running even if `gzserver` dies, so "the launch is alive" does not mean
"the system is alive". Check with `pgrep -xc gzserver`, and if `ros2 topic list` shows no `/clock`, the
simulator is gone.

---

## 6. Troubleshooting

| symptom | cause | fix |
|---|---|---|
| `free(): invalid pointer`, `gzserver` exit −6 | intermittent, this environment | retry (§5) |
| `Unable to start server[bind: Address already in use]`, `gzserver` exit 255 | a previous `gzserver` still holds port 11345 — restarting quickly is enough to cause this | kill it (`pkill -x gzserver gzclient`), **wait for the port to be free**, or sidestep it entirely with `export GAZEBO_MASTER_URI=http://127.0.0.1:11346` (any free port) |
| the GUI starts but nothing moves | nothing is publishing on the chain root's reference | run the drill (§3.3) |
| the GUI window is black | GL stack of the host (this machine renders through Mesa `svga` and prints `context mismatch in svga_surface_destroy`) | `LIBGL_ALWAYS_SOFTWARE=1`, or watch headless and measure instead |
| recorder: `no diagnostics from tp_leaf` | the simulator died after the launch reported ready, or discovery is slow | check `pgrep -xc gzserver`; the recorder already waits up to 45 s |
| `ros2 node list` empty but topics exist | stale `ros2` daemon | use `--no-daemon`: `ros2 topic list --no-daemon` |
| `Couldn't parse parameter override rule` | the plugin re-passes the URDF as a command-line override, and rcl parses it as YAML | the launch file already strips the XML declaration and comments and collapses it to one line; keep the URDF free of `--` inside comments |
| `parser error ... Double hyphen within comment` | XML comments may not contain `--` | remove any `--` from the URDF comment text |
| declaring a `velocity` command interface aborts gzserver | this Humble `GazeboSystem` is position-only | keep `<command_interface name="position"/>`; the demo sets `position_command: true` so the controller emits a setpoint |
| joint rails to ±3.14 | a rate law driving a position interface (`position := target − position`) | set `position_command: true` on the cascade controllers, as the demo YAML does |
| all levels report `lag 0` even in single pass | the baseline was taken during the start-up transient, or the stamps are missing | the recorder waits for the joint to settle; verify the period line prints `10.00 ms` (if it prints nothing, the messages carry no stamp) |
| a controller has no `/tp_*/cycle_diagnostics` topic | a publisher created in `on_configure` is destroyed when a chained parent activates its child (the child is deactivated and re-activated to enter chained mode) | create active-phase resources in `on_activate`; the example controller does |

---

## 7. What is actually being measured

`record_chain_lag.py` subscribes to the three cascade controllers' `~/cycle_diagnostics` topics. Each
message is `[cycle, estimate, child_published, stamp_s]`, where `stamp_s` is the **manager's time for that
cycle** — shared by every member, which is what makes the three series comparable. The controllers are
activated at different moments, so their own cycle counters are *not* a common time base.

The lag is counted in control cycles from the **leaf's** first movement, so the plant's dynamics cannot
contaminate it: the step is detected on each level's own estimate, relative to the same origin.
