# Trained-Policy Inference (FFW SG2 — SmolVLA / GR00T / Diffusion)

Rolling out trained policies (and composed variants) through the Cyclo
Intelligence stack. Policies publish joint-space actions directly to the
follower's arm controllers — **arms + grippers only**; no Cartesian
controller, no leader, no eef bridge (those belong to teleop and the LLM
agent — see `TABLETOP_TELEOP_README.md` / `LLM_AGENT_README.md`).

> **ONE-ARMED MODE (left arm only) is the default on the `one-armed` branches** of
> `ai_worker`, `cyclo_intelligence` and `cyclo_control` (since 2026-11-27; the right shoulder
> servo, ID 1, is faulty). The right arm is never powered. It stays wherever it was left and
> no pose is assumed for it. What this changes for policy inference:
>
> | | One-armed (default) | Two arms (only when the right arm is healthy) |
> |---|---|---|
> | Follower bringup | `ffw_sg2_follower_ai.launch.py` (left arm only) | `ffw_sg2_follower_ai_both_arms.launch.py` |
> | Robot type in the UI | `ffw_sg2_rev1_left` | `ffw_sg2_rev1` |
> | Policy action | 8 numbers: left arm (7) + left gripper | 16 numbers: both arms + grippers |
> | Policy state | 14 numbers: left arm + gripper (8), head (2), lift (1), odometry (3) | 22 numbers |
> | Cameras | `cam_left_head`, `cam_left_wrist` (`cam_right_head` is also recorded) | + `cam_right_wrist` |
> | Re-home | `go_to_initial_state.py` (default `--arms left`) | add `--arms left,right` |
>
> - **Only left-arm policies run in this mode.** On `ffw_sg2_rev1_left` the left arm is the only
>   action group, so a policy must output exactly 8 numbers; anything else is refused at LOAD
>   (the old two-arm checkpoints are refused too, usually first for needing `cam_right_wrist`).
>   A policy on this robot type cannot command the head, lift or base.
>   Policies must be retrained on data recorded with `ffw_sg2_rev1_left`.
> - Do NOT load old two-arm checkpoints with the stock robot type `ffw_sg2_rev1` on the one-armed
>   robot: they would run on a right-arm state they never saw in training.
> - No policy code changed; this all comes from the robot config file.
> - `ffw_sg2_rev1_left` appears in the UI only after it is installed in the `cyclo_intelligence`
>   container and the orchestrator is restarted (see "One-armed deployment status" at the end).
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

Backends and their containers (one policy loaded at a time per backend):

| backend | container | serves |
|---|---|---|
| lerobot | `lerobot_server` | SmolVLA, Diffusion Policy (+ lerobot composites) |
| groot   | `groot_server`   | GR00T N1.7 (+ groot composites) |

Enter containers with `cd ~/<repo> && ./docker/container.sh enter`.
`/workspace` in cyclo containers = `~/cyclo_intelligence/docker/workspace`.

---

## Prereqs

- Robot powered, **E-STOP A pressed** (torque on), E-STOP in hand.
- Zenoh daemon up (`docker ps | grep zenoh`).
- Backend container RUNNING — after a robot reboot they stay DOWN:
  ```bash
  docker start lerobot_server    # and/or groot_server
  ```
  (UI shows the backend as "missing" when its container is stopped.)
- Web UI: `http://192.168.6.2:7080` (robot LAN IP — NOT localhost).

## Start sequence

### T1 - Follower at the tabletop start pose (ai_worker container)
Put the unpowered right arm in a safe resting position first (it is not moved or held).
```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py \
  init_position_file:=tabletop_manip_left_arm_position.yaml
```
This is the one-armed bringup: only the left arm, head and lift go to the start pose, and the
right-arm servos stay unpowered (still read for `/joint_states`).
No other commander may run (no leader teleop, no vr controller, no movej).

### T2 - Load + run from the Web UI
Inference page -> robot type `ffw_sg2_rev1_left` -> select backend -> browse to the model
folder -> set the TASK STRING -> LOAD -> START. Stop with STOP; Clear = STOP + UNLOAD.

Model folders (host `~/cyclo_intelligence/docker/workspace/model/`,
container `/workspace/model/`), grouped by task:
```
model/lerobot/blue-cylinder/   pick L/R (aug, wrist), handover-aug, pick+handover composites
model/lerobot/push-tape/       wrist + cam3 pairs and composites
model/lerobot/soft-toy/        wrist pair + composite
model/lerobot/bread/           3cam trio + pan+tray(+container) composites (joint & stitch)
model/lerobot/wave/            wave-right state-policy demo
model/groot/                   groot singles + composites (flat)
```
In the UI's folder browser, descend INTO the task folder and select the
model directory itself.

Task strings (language-conditioned models; diffusion ignores them):
- cylinder / handover models: `Pick up the blue cylinder`
- push-tape models: `Push the red tape`
- soft-toy: left `Move the soft toy to left`, right `Move the soft toy to right`
  (composite: pick one, or try "Move the soft toy")
- bread: trained from inference-merged data — check the dataset card.

## Between episodes — re-home

Disarm nothing (no controller here), just stop the policy (UI STOP), then:
```bash
# cyclo_intelligence container
python3 /workspace/scripts/go_to_initial_state.py \
  --model-dir /workspace/poses/tabletop_manip        # add --include-lift if needed
```
By default (`--arms left`) it moves the LEFT arm and the head (add `--skip-head` to leave the
head alone); the right arm is never commanded.
Gripper stuck closed: see "Gripper open (manual)" in TABLETOP_TELEOP_README.md.
Every model dir carries its own `initial_state.json` (dataset mean start
pose) — the homing/reset tools read whatever `--model-dir` you point them at.

---

## Composites (composed policies)

A composite "model dir" holds `composite.json`:
```json
{"models": ["<memberA>", "<memberB>"], "weights": [0.5, 0.5], "mode": "joint"}
```
- `mode: "joint"` = score composition (weighted velocity/eps sum per denoise
  step, one shared trajectory); `"stitch"` = output composition (native per-
  policy chunks combined per-dim). Members resolve relative to the composite's
  parent folder — keep composites in the same task folder as their members.
- Edit weights in the json, then **Clear -> Load** in the UI to apply. The
  `config.json` inside a COMPOSED dir is a routing stub — never edit it; real
  inference settings live in the MEMBER dirs.
- Members must share normalization stats (sharednorm / same-group training) —
  verify before composing new pairs.
- NOTE: composition code lives on the `compose` branch of cyclo_intelligence.
  On `tabletop-manip` (the llm-as-policy branch) composite dirs will NOT load.

## Knobs

- `num_inference_steps` (diffusion) / `num_steps` (SmolVLA) /
  `num_inference_timesteps` (GR00T) in the MODEL dir's `config.json`;
  re-Load to apply. Diffusion ships at 10 here (100 blows the 5 s timeout).
- `/workspace/noise_idx.txt` (lottery-ticket / golden-ticket noise) is DISABLED in the lerobot
  engine on `tabletop-manip` and `one-armed`: the file is never read and every chunk uses
  fresh random noise. To use it, uncomment the block in `lerobot_engine/prediction.py`.
- Chunk pipeline (container env, recreate to apply): `INFERENCE_HZ` 15,
  `CONTROL_HZ` 100, `REFILL_MARGIN_S` 0.2 (0.4 made staleness worse).
- `noproprio.flag` in a model dir (or "noproprio" in its name) makes the
  lerobot engine feed zeroed `observation.state`.

## Notes / gotchas

- **Code changes** in the mounted engines (`lerobot_engine/`, groot
  `runtime/`, `lerobot` / `Isaac-GR00T` libs) need a container restart
  (`docker restart lerobot_server`) — UI Clear does NOT reload code.
- Container **env** changes (compose/.env) need a RECREATE
  (`docker compose -f docker-compose.yml up -d <service>`), not a restart.
- A loaded policy holds publishers on the follower arm topics even when
  stopped — restart the backend container before running teleop or the LLM
  agent, or they will see "2 publishers" warnings.
- GR00T models are ~12 GB on the Jetson's shared RAM: a loaded groot
  composite (2 models) eats ~22 GB — UNLOAD (Clear) when done or everything
  else starts swapping.
- Inference recordings from the UI land in `/workspace/rosbag2/
  Task_<stamp>_inference_MCAP`; empty ones (0 files) are failed takes.
- Model inventory lives on HF (`omkarpatil`); re-download anything deleted
  locally with `huggingface_hub.snapshot_download` into the right task folder.


## One-armed deployment status (2026-11-27)

- **Bringup (ai_worker):** installed in the container and tested with mock hardware only. The
  first run on the real servos is still to do; keep the E-STOP in hand.
- **Robot types `ffw_sg2_rev1_left` / `ffw_sg2_rev1_left_eef` (cyclo_intelligence):** files exist
  on the `one-armed` branch. The policy containers read them directly. The UI lists a robot
  type only once its config is in the orchestrator's installed `shared` package and the
  orchestrator has been restarted.
- **`lerobot_server` / `groot_server`:** restart the container to pick up code on this branch.
- **Data for retraining:** record with robot type `ffw_sg2_rev1_left` so datasets carry the
  14-number state and 8-number action the left-arm policies will use.
