# LLM-as-Policy Inference (FFW SG2, llm_agent backend)

> **ONE-ARMED MODE (left arm only) is the default on the `one-armed` branches** (since
> 2026-11-27; the right shoulder servo is faulty). The LLM drives the LEFT arm only. The right
> arm is never powered or commanded, it stays wherever it was left, and no pose is assumed for
> it; the model is told it is a fixed obstacle of unknown position.
>
> | | One-armed (default) | Two arms |
> |---|---|---|
> | Follower bringup | `ffw_sg2_follower_ai.launch.py` | `ffw_sg2_follower_ai_both_arms.launch.py` |
> | Controller setting | `enable_right_arm: false` (code default) | `enable_right_arm: true` in the `vr_controller` section |
> | Robot type | `ffw_sg2_rev1_left_eef` (default of `llm_agent_ctl.py`) | `--robot-type ffw_sg2_rev1_eef` |
> | Goal bridge | `eef_goal_bridge.py` (default `--arms left`) | `--arms left,right` |
> | Goal row | `(1, 7)` on `/l_goal_eef` | `(1, 14)` on `/l_goal_eef` + `/r_goal_eef` |
> | Cameras the model sees | `cam_left_head`, `cam_left_wrist` | + `cam_right_wrist` |
> | Re-home | `go_to_initial_state.py` (default `--arms left`) | `--arms left,right` |
>
> **Needs the one-armed `cyclo_control` build in the ai_worker container.** With the old
> controller build, `vr_controller` waits for a right-arm goal and never arms
> ("Waiting for goal poses: right=MISSING").
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

A frontier LLM (via inspect-robots) drives the arm through **end-effector
goals**: the engine observes (2 cameras + FK pose), the model makes one
`move_by`/`done`/`give_up` tool call per step, and cyclo_control's
`vr_controller_node` does the IK. One LLM call ≈ one small motion (≤0.10 m,
≤0.5 rad), rate-limited to ≤0.05 m/s by the goal bridge.

```
llm_agent_server            cyclo container             ai_worker container
 engine: obs -> LLM ->  ->  eef_goal_bridge.py:     ->  vr_controller_node
 absolute (1,7) goal row    20 Hz rate-limited          (controller_type:=vr,
 on /l_goal_eef             PoseStamped + gripper       enable_right_arm false)
```

Design doc: `~/cyclo_intelligence/cyclo_brain/policy/llm_agent/README.md`.
Everything below: enter containers with `cd ~/<repo> && ./docker/container.sh enter`.
NOTE: `/workspace` differs per container — ai_worker's is `~/ai_worker/docker/workspace`,
cyclo's is `~/cyclo_intelligence/docker/workspace` (scripts + logs live in the latter).

Files (cyclo container `/workspace` unless noted):
- `/workspace/scripts/llm_agent_episode.sh`  — ONE-SHOT episode: load+start+record+ARM
- `/workspace/scripts/llm_agent_ctl.py`      — manual LOAD/START/STOP/UNLOAD/instruction
- `/workspace/scripts/eef_goal_bridge.py`    — goal rows -> vr_controller (rate limiter)
- `/workspace/scripts/llm_agent_video.py`    — render step frames to MP4 (harness encoder)
- `/workspace/scripts/go_to_initial_state.py`— re-home between episodes
- `/workspace/llm_agent/logs/<ts>/`          — per-episode outputs (see below)
- `~/cyclo_intelligence/docker/.env`         — API keys + knobs (container recreate to apply!)

Prereqs: robot powered, **E-STOP A pressed** (torque on), zenoh daemon up,
`llm_agent_server` container **healthy** (`docker ps | grep llm_agent`).
After a robot reboot it stays DOWN (`restart: no`) — bring it up with:
```bash
cd ~/cyclo_intelligence/docker && docker compose -f docker-compose.yml up -d llm_agent
```
API keys live in `docker/.env` (`OPENAI_API_KEY`, `GEMINI_API_KEY`,
`ANTHROPIC_API_KEY`). Any `.env` change needs the `up -d llm_agent` above
(env is baked at container CREATION — `docker restart` is NOT enough).

---

## Start sequence

Terminals T1/T2 in the **ai_worker** container, T3/T4 in **cyclo_intelligence**.

### T1 - Follower at the tabletop start pose (left arm only)
Put the unpowered right arm in a safe resting position first, clear of the left arm's workspace.
```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py \
  init_position_file:=tabletop_manip_left_arm_position.yaml
```
(Dry run: add `use_mock_hardware:=true`.)

### T2 - Cartesian controller in VR mode (tabletop tuning)
```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
  controller_type:=vr config_file:=/workspace/ai_worker_config_amp.yaml
```
Waits on `/reactivate` — expected. It logs "enable_right_arm is false: right arm is not
commanded and is modelled at its measured pose." With that, the startup check and the collision
model use the right arm's measured pose, wherever it is.

### T3 - Goal bridge
```bash
python3 /workspace/scripts/eef_goal_bridge.py
```
Expect `bridge up: 20 Hz ... workspace [0.25, 0.75, -0.55, 0.55, 0.75, 1.2]; arms ['left']`
and NO "has 2 publishers" warning (if warned: a leader/movej/policy is still
commanding the arms — stop it first).

### T4 - Run an episode (all-in-one)
```bash
bash /workspace/scripts/llm_agent_episode.sh "TASK STRING" [model]
# e.g.
bash /workspace/scripts/llm_agent_episode.sh \
  "pick up the red cube and place it in the blue basket." openai/gpt-6.1-sol
```
The script: stop+unload any old policy -> LOAD(task) -> START -> starts a
**rosbag video recording** into the episode dir -> **ARMS after a 5 s
countdown** (Ctrl+C before that aborts) -> tails the engine log.
**Ctrl+C ends the episode**: disarm, stop+unload (this writes the
transcript), finalize the bag.

Timing after arming: ~3 s controller startup check + ~8 s slow-start + the
engine's 12 s grace, then the first LLM call. Cycle: model thinks (10-60 s,
arms still) -> small motion -> settle -> next call. **E-STOP in hand.**

Models (prefix = provider): `openai/gpt-6.1-sol`, `openai/gpt-6-luna`,
`google/gemini-4-argon`, `anthropic/...`. The engine picks the right API
wire per provider automatically (OpenAI -> responses, others -> chat).

### Manual control (instead of the episode script)
```bash
python3 /workspace/scripts/llm_agent_ctl.py load --model openai/gpt-6.1-sol \
    --instruction "TASK" --publish        # robot type defaults to ffw_sg2_rev1_left_eef
python3 /workspace/scripts/llm_agent_ctl.py start --publish
ros2 topic pub --once /reactivate std_msgs/msg/Bool "{data: true}"
# mid-episode task change:
python3 /workspace/scripts/llm_agent_ctl.py instruction --instruction "NEW TASK"
# end:
python3 /workspace/scripts/llm_agent_ctl.py stop     # pause; model stays loaded
python3 /workspace/scripts/llm_agent_ctl.py unload   # finalize episode + transcript
```
Model swap needs the FULL cycle: `stop` -> `unload` -> `load` -> `start`
("policy already loaded - UNLOAD first" otherwise). After every (re)load,
re-publish `/reactivate true` — the fresh listener only sees new messages.

---

## Between episodes

1. Disarm: `ros2 topic pub --once /reactivate std_msgs/msg/Bool "{data: false}"`
2. Re-home (cyclo container):
   `python3 /workspace/scripts/go_to_initial_state.py --model-dir /workspace/poses/tabletop_manip`
   (moves the left arm and the head, add `--skip-head` to leave the head; the right arm is
   never commanded)
3. **Restart the T3 bridge.** It latches the last reference; after a re-home
   the controller refuses to arm with "Startup mismatch" against the stale
   goal. Fresh bridge publishes nothing until the engine's measured-pose
   seed row, so the startup check then passes.
4. Run the next episode (the episode script handles the rest).

---

## Episode outputs (`/workspace/llm_agent/logs/<timestamp>/`, cyclo container)

| file | what |
|---|---|
| `episode.json`     | model, instruction, seed goals, finish reason |
| `steps.jsonl`      | per step: state, action delta, goal row, latency (live) |
| `frames/`          | JPEGs the model saw, per step/camera |
| `frames_npy/`      | same frames, raw (harness FrameStore format) |
| `transcript.jsonl` | full LLM transcript — written at episode END (done/give_up/unload) |
| `rosbag/`          | true-motion recording (3 cams + joints + goals), episode script only. The right wrist camera is still recorded; it shows wherever the unpowered arm points |
| `videos/`          | after rendering: `python3 /workspace/scripts/llm_agent_video.py <dir> [--fps 2]` |

Live view while running: `docker logs -f llm_agent_server`.

---

## Safety layers & knobs

Between the LLM and the motors: per-call cap (0.10 m / 0.5 rad, `spec.py`) ->
workspace box clamp, enforced in BOTH engine and bridge -> bridge rate limit
(0.05 m/s, 0.3 rad/s, gripper 1.0 rad/s) -> vr_controller reference checker +
slow-start -> manual arming -> E-STOP.

- Workspace box (base_link, m): `LLM_AGENT_WORKSPACE` in `docker/.env` AND the
  bridge `--workspace` default — **keep them identical**. Current zmin 0.75 =
  tabletop finger height 0.901 minus 6 in. It does NOT follow the lift: if the
  lift moves from -0.0852, recompute zmin.
- Bridge speeds: `--linear-mps/--angular-rps` flags; if changed, mirror via
  `LLM_AGENT_BRIDGE_LINEAR_MPS/_ANGULAR_RPS` so the engine's settle wait stays right.
- Engine env (in docker-compose/.env, recreate to apply): `LLM_AGENT_CAMERAS`,
  `LLM_AGENT_MAX_LLM_CALLS` (60), `LLM_AGENT_EFFORT` (medium),
  `LLM_AGENT_ARM_GRACE_S` (12), `GET_ACTION_TIMEOUT_S` (180).
- The embodiment text the model reads is built in `llm_agent_engine/spec.py`: `DOCS` for two
  arms, `docs_for()` for one arm (it says the other arm is off, at an unknown position, and a
  fixed obstacle);
  engine code changes need `docker restart llm_agent_server` (code is bind-mounted).

---

## Notes / gotchas
- Only ONE commander: never run leader teleop / movej / another policy with
  this stack (the bridge's publisher check catches it).
- "Startup mismatch. Waiting." from vr_controller = stale bridge reference
  (see Between episodes) — it is the safety gate working, not a fault.
- Episode script prints nothing and exits = see its usage line; hangs at
  start = llm_agent_server down (robot reboot) — `up -d llm_agent`.
- HTTP 404/400 from the LLM: wrong model id for your key, or wire mismatch —
  model ids must be prefixed `openai/`, `google/`, `anthropic/`.
- KNOWN ISSUE: the engine's settle wait can return before motion starts, so
  the model may see a stale pose and re-issue a move (stacked goals — how an
  episode once hit joint limits). Until fixed: prefer models' waiting, and
  watch the first moves closely.
- Grippers: RH-P12-RN pads close to a ~1 cm gap at full command — objects
  thinner than ~1 cm can't be gripped.
- One-armed: tasks must be doable with the left arm alone, and the task string should not ask
  for the right arm. The model cannot move the right arm out of the way; if it blocks the
  workspace, move it by hand with the bringup stopped.
- One-armed: replay files (`replay:<file.json>`) may only use `left_*` labels or 7-value rows.
  Use `/workspace/llm_agent/dry_run_left.json` for dry runs; the older `dry_run.json` has
  right-arm rows and is refused at LOAD on the one-armed robot type.
- After pulling engine changes, `docker restart llm_agent_server` before the next episode: the
  server keeps the code it started with.
