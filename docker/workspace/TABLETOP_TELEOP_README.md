# Tabletop Cartesian Teleop (FFW SG2, LG2 leader)

Leader-driven **Cartesian** teleop (`controller_type:=leader`) tuned for table-top
manipulation. Hands follow the leader; Shoulder Yaw is biased to a fixed posture;
grippers follow; motion is 1:1 (no amplification). movej is **not** used here.

> **ONE-ARMED MODE (left arm only) is the default on the `one-armed` branches** (since
> 2026-11-27; the right shoulder servo is faulty). Only the LEFT follower arm follows the leader.
> The right arm is never powered or commanded; it stays wherever it was left and no pose is
> assumed for it. The controller models it at its measured pose, but its collision check skips
> most left-arm vs right-arm pairs, so do not rely on it to keep the arms apart (see rule 1).
>
> | | One-armed (default) | Two arms |
> |---|---|---|
> | Follower bringup | `ffw_sg2_follower_ai.launch.py` | `ffw_sg2_follower_ai_both_arms.launch.py` |
> | Controller setting | `enable_right_arm: false` (code default) | `enable_right_arm: true` in every controller section of the config |
> | Homing button / reset | default `--arms left` | `--arms left,right` |
>
> - You still wear both leader arms and hold both triggers to engage; the right leader arm is
>   simply ignored by the controller.
> - **Needs the one-armed `cyclo_control` build in the ai_worker container.** With the old
>   build the controller waits for the right arm to match its goal and never arms.
> - Record demonstrations with robot type `ffw_sg2_rev1_left` so datasets hold the left arm only.
> - This guide's Cartesian teleop is the supported one-armed teleop. DIRECT leader-to-follower
>   teleop (`ffw_sg2_ai`, the Cyclo Manager "Leader") is not: its "joints synced" test averages
>   both arms, so with a limp right follower arm it may never report synced and the left arm
>   stays in its slow catch-up mode.
>
> **Two rules for one-armed operation:**
> 1. **Secure the right arm well clear of the left arm, the table edge and the lift's travel
>    before every bringup, homing or run.** Nothing checks where it is: the bringup, the reset
>    scripts, policies and replays move the left arm and lift without knowing, and the
>    controller's collision model does not cover most left-arm vs right-arm contacts.
> 2. **To cut torque use the E-STOP or kill the bringup.** The software torque-off service does
>    nothing in one-armed mode.
>
> **Before the first bringup after any container restart, recreate or image update, run**
> `~/ai_worker/tools/one_armed_container_check.sh` **on the host.** The one-armed bringup and the
> one-armed `cyclo_control` build live only inside the `ai_worker` container; a recreated container
> silently goes back to the stock two-arm bringup. The check says what is missing and
> `--install` puts the bringup files back.
> Starting the robot from the Cyclo Manager UI is safe for the right arm (the bringup ignores the
> `ros2_control_type` the UI sends), but the UI may reuse an old start-pose file such as
> `pack_position.yaml`: check the arguments it shows before pressing start.

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

### T1 - Follower at the tabletop start pose (left arm only)
Put the unpowered right arm in a safe resting position first, clear of the left arm's workspace.
```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py \
  init_position_file:=tabletop_manip_left_arm_position.yaml
```
Wait until the left arm, lift and head reach the start pose and the arm settles.
(Add `launch_cameras:=false launch_lidar:=false` if you don't need them.)

### T2 - Cartesian controller (leader mode) with the tabletop tuning
```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
  controller_type:=leader config_file:=/workspace/ai_worker_config_amp.yaml
```
It logs `motion_scale`, `Posture bias: arm_*_joint3 ...`, "enable_right_arm is false: right
arm is not commanded and is modelled at its measured pose.", then
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
  --model-dir /workspace/poses/tabletop_manip        # homes the LEFT arm only (default --arms left)
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

## Gripper open (manual)

Force a gripper open (e.g. it's clamped on an object after a run). Stop/Clear
any policy and disarm teleop first, or the command is immediately overridden.
Run in any container shell; only the named gripper joint moves (1 s smooth open;
0.0 = fully open, ~1.1 = fully closed):
```bash
# LEFT gripper
ros2 topic pub --once /leader/joint_trajectory_command_broadcaster_left/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_l_joint1], points: [{positions: [0.0], time_from_start: {sec: 1}}]}"

# RIGHT gripper
ros2 topic pub --once /leader/joint_trajectory_command_broadcaster_right/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_r_joint1], points: [{positions: [0.0], time_from_start: {sec: 1}}]}"
```

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
