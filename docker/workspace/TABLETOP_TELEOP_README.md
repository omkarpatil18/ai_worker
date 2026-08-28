# Tabletop Cartesian Teleop (FFW SG2, LG2 leader)

Leader-driven **Cartesian** teleop (`controller_type:=leader`) tuned for table-top
manipulation. Hands follow the leader; Shoulder Yaw is biased to a fixed posture;
grippers follow; motion is 1:1 (no amplification). movej is **not** used here.

All paths below are inside the Docker containers. Enter a container with:
`cd ~/<repo> && ./docker/container.sh enter`  (repo = `ai_worker` unless noted).

Files (host, git-ignored under `ai_worker/docker/workspace/` == `/workspace` in the container):
- `/workspace/ai_worker_config_amp.yaml`         — controller tuning (loaded by the controller)
- `/workspace/ffw_lg2_leader_movej.launch.py`    — leader launch (remaps broadcaster -> raw topics)
- `/workspace/ffw_lg2_leader_movej_controllers.yaml` — leader hardware-controller params (sync_threshold)
- `/workspace/scripts/go_to_initial_state.py`    — home/reset tool (runs in the cyclo_intelligence container)
- `/workspace/poses/tabletop_manip/initial_state.json` — the tabletop start pose

Prereqs: robot powered, **E-STOP A pressed** (torque on), Zenoh daemon running
(`docker ps | grep zenoh_daemon`). Keep the E-STOP in hand throughout.

---

## Start sequence

Use SEPARATE terminals (T1..T3), each entered into the `ai_worker` container.

### T1 - Follower at the tabletop start pose
```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py \
  init_position_file:=tabletop_manip_position.yaml
```
Wait until the arms/lift/head reach the start pose and the arm settles.
(Add `launch_cameras:=false launch_lidar:=false` if you don't need them.)

### T2 - Cartesian controller (leader mode) with the tabletop tuning
```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
  controller_type:=leader config_file:=/workspace/ai_worker_config_amp.yaml
```
It logs `motion_scale`, `Posture bias: arm_*_joint3 ...`, then
`waiting for reactivate topic '/reactivate' to become true` - expected.

### T3 - Leader (remapped launch)
```bash
ros2 launch /workspace/ffw_lg2_leader_movej.launch.py
```

### Engage
1. Wear the leader, match the robot's current pose.
2. Hold BOTH hand triggers for ~2 s (leader "Auto mode ACTIVATED").
3. Arm the controller (any container shell):
   ```bash
   ros2 topic pub --once /reactivate std_msgs/msg/Bool "{data: true}"
   ```
   Wait ~3 s (startup pose check) + ~8 s slow-start ramp -> full tracking.
   Move slowly at first. Grippers follow the leader grip.

### Pause / stop
- Pause tracking: `ros2 topic pub --once /reactivate std_msgs/msg/Bool "{data: false}"`
  (releasing the triggers only freezes the goal; publish `false` to actually disarm.)
- Stop: `Ctrl+C` in T3 (leader) first, then T2 (controller). T1 last.

---

## Homing button on the leader (long-press left joystick tact)

A background node maps the leader's **unused left-tact long-press** to "arms
return to the tabletop pose". Run it once (in the **cyclo_intelligence**
container) alongside teleop, in its own terminal:
```bash
python3 /workspace/scripts/homing_button_node.py \
  --model-dir /workspace/poses/tabletop_manip
```
Then, to home: **disengage the leader**, and **long-press the LEFT joystick
tact** (event `left_long_time`). The node briefly disarms (`/reactivate false`)
so nothing fights the move, sends one timed move of the ARMS to the tabletop
pose over 5 s, then **re-arms automatically** (`/reactivate true`). No keyboard:
just **re-engage the leader** (hold both triggers, match the robot pose) and
teleop resumes. Pass `--no-rearm` to leave it disarmed instead.

- Arms only by default. Head/lift are streamed live by the leader joystick at
  100 Hz, so `--include-head` / `--include-lift` only stick if the leader stack
  is fully stopped (not just triggers released).
- Grippers are left as-is (not in the pose file).
- Change the button with `--trigger-event {left,right,left_long_time,right_long_time}`.
  `left`/`right` short presses are used by cyclo_intelligence for record
  start/cancel — don't reuse those while recording. Both long-presses are free.
- Target pose = `initial_state.json` in `--model-dir` (same file the reset
  script uses — single source of truth).

---

## Re-home (send arms back to the tabletop start pose)

Run in the **cyclo_intelligence** container (`cd ~/cyclo_intelligence && ./docker/container.sh enter`).
The leader must be OFF (its joystick controller streams head/lift otherwise).
```bash
python3 /workspace/scripts/go_to_initial_state.py \
  --model-dir /workspace/poses/tabletop_manip --include-lift
```
It prints the planned move, asks y/N, moves over ~5 s, then reports per-joint error.

---

## Tuning (settled 2026-08-28)

Edit `/workspace/ai_worker_config_amp.yaml`, then relaunch **T2 only**.

| key | value | meaning |
|---|---|---|
| `motion_scale` | 1.0 | no amplification (1:1). >1 pushes goals outward -> worse near-body reach |
| `weight_elbow_position` | 0.0 | KEY: frees the elbow to fold so the hand can reach in (stock 8) |
| `posture_positions` | [-2.3, 2.3] | Shoulder Yaw target (L, R). MUST match the home pose's j3 or the arm lurches on engage |
| `posture_weights` | [3.0, 3.0] | Shoulder-Yaw hold strength (0 = free/wanders, 10 = stiff) |
| `kp_posture` | 5.0 | return speed toward the yaw target |
| `weight_accel` | 0.1 | soft acceleration limit (reduce jerk); raise for smoother/laggier |
| `remap_to_follower_shoulder` | true | anchor goals on the follower shoulder (fixes lateral offset) |
| `publish_gripper` | true | grippers follow in leader mode |

If you change the Shoulder-Yaw target, also update the home pose so they agree:
- `ai_worker/ffw_bringup/config/ffw_sg2_rev1_follower/tabletop_manip_position.yaml`
  (arm_l/arm_r `home` j3), then rebuild: `colcon build --packages-select ffw_bringup`
- `cyclo_intelligence/docker/workspace/poses/tabletop_manip/initial_state.json` (arm_*_joint3)

---

## Notes / gotchas
- Never run `ffw_sg2_ai` (all-in-one) here - it launches the UNremapped leader,
  which double-publishes on the follower arm topics. Use the T1/T3 split above.
- Only ONE thing may command the arms at a time (no movej/policy while teleoperating).
- The controller code is the `cyclo_control` fork branch `tabletop-cartesian-teleop`
  (omkarpatil18), built into the ai_worker container. The tuning values above live
  only in this workspace config, not in the committed code (which defaults them OFF).
- Web UI (Cyclo Intelligence/Manager): open `http://192.168.6.2:7080`, NOT localhost.
