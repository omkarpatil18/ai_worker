# ROBOTIS AI Worker — Documentation Digest for Claude

Condensed from every page of https://docs.robotis.com/docs/systems/aiworker/ (53 pages, crawled 2026-08-23).
Purpose: give a fresh Claude instance running **on the robot's Jetson** full working context without re-reading the site.

---

## 0. THIS robot (local facts, verified 2026-08-23)

| Item | Value |
|---|---|
| Hostname | `ffw-SNPR48A1025` (mDNS: `ffw-SNPR48A1025.local`) — serial suffix **1025** |
| User / password | `robotis` / `root` (ROBOTIS default) |
| JetPack | L4T R36.4.3 (JetPack 6.2), Jetson AGX Orin 32GB |
| `ai_worker` repo | `~/ai_worker`, branch `jazzy`, package version **2.2.5** (`2.2.5-44-gb532d61`) |
| Running containers | `ai_worker` (robotis/ai-worker:2.2.5), `zenoh_daemon`, `cyclo_intelligence` (1.3.0), `cyclo_manager` + `cyclo_manager_ui` (1.0.1), `novnc-server`, `robotis-applications` (0.1.1), legacy `physical_ai_manager` + `physical_ai_server` |
| Other repos in `~` | `cyclo_intelligence`, `physical_ai_tools` (legacy), `robotis_applications`, `fw` |
| Local uncommitted change | `ffw_bringup/config/common/rs_serial.yaml` (RealSense serials — **must be preserved** across package updates) |
| Launch files present | bg2/sg2/sh5/bh5/f1/f2 follower + gazebo variants, `ffw_lg2_leader_ai`, `ffw_mobile_base_ai`, `ffw_pack`, camera/lidar launches |

**Do NOT run `apt upgrade` on the Orin** (docs: it breaks robot functionality).

---

## 1. What the AI Worker is

Semi-humanoid "Physical AI" robot from ROBOTIS: mobile swerve-drive base (SG2) or static base (BG2), dual 7-DOF arms, 1-DOF grippers (RH-P12-RN) or dexterous hands (SH5/BH5, in prep), 2-DOF head, 1-DOF lift, Jetson AGX Orin 32GB, ROS 2 Jazzy in Docker, fully open source (Apache 2.0). Learns via teleop-collected imitation learning data; supports RL in sim. Naming: **FFW** = product family; **F**ollower = robot body, **L**eader = wearable teleop device.

Model lineup:
- **FFW-SG2** — Swerve base + Grippers (this robot's launch names use `sg2`). Battery 25V/80Ah ×2. 25 DOF total.
- **FFW-BG2** — Bench/static, Grippers, AC via SMPS (Korea only). 19 DOF.
- **FFW-SH5 / BH5** — Hand (20-DOF/hand) variants, in preparation.
- **FFW-LG2** — Leader with grippers/joysticks (22 DOF). **FFW-LH5** — leader with hands (60 DOF), in prep.

---

## 2. Hardware specs (key numbers)

- Dimensions SG2: 604×602×1623 mm, 90 kg. BG2: 604×564×1607 mm, 85 kg.
- Arm reach 641 mm to wrist; payload 3 kg nominal / 5 kg peak per arm (6/10 kg dual).
- Mobile: swerve drive, 3 wheels (triangular), 1.5 m/s. Joints `{left,right,rear}_wheel_{steer,drive}`; steer ±90°.
- Actuators (single RS-485 bus, 4 Mbps, Protocol 2.0, via U2D2):
  - Arm J1–3: DYNAMIXEL-Y YM080-230-R099-RH; J4–6: YM070-210-R099-RH; J7: DYNAMIXEL-P PH42-020-S300-R
  - Gripper RH-P12-RN (0–107.6 mm, 500 g, 5 kg payload, current-based position control)
  - Head pitch XH540-V150-R, head yaw XH430-V210-R; Lift YM080-230-B001-RH; Wheel steer YM070-210-R051-RH
- Sensors: head **ZED Mini** (stereo RGBD + IMU, 102°×57°, 0.1–9 m), 2× wrist **RealSense D405** (87°×58°, 7–50 cm), SG2 only: 2× **LakiBeam 1** 2D LiDAR (270°, 25 m, Ethernet UDP/USB).
- SG2 internal router: TP-Link AC1200; Wi-Fi SSID `AIWORKER<number>`, password = SSID. Host Ethernet up to 100 Mbps.
- Joint IDs / names / ranges:

| ID | Joint | Range |
|---|---|---|
| 1–7 | `arm_r_joint1..7` (shoulder pitch/roll/yaw, elbow, wrist yaw/pitch/roll) | J1 ±180°, J2 −190..10°, J3 ±180°, J4 −170..65°, J5 ±180°, J6 ±105°, J7 −120..90° |
| 8 | `gripper_r_joint1` | 0–107.6 mm |
| 31–37 | `arm_l_joint1..7` | mirrored (J2 −10..190°, J7 −90..120°) |
| 38 | `gripper_l_joint1` | 0–107.6 mm |
| 61 / 62 | `head_joint1` pitch / `head_joint2` yaw | −50..30° / ±20° |
| 81 | `lift_joint` | 0–500 mm |
| 101/103/105 | wheel steer (see multi-turn guide) | ±90° |

- Ports (SG2 back): WAN (internet), LAN (SSH/remote), USB, HDMI, Charge port. BG2 has no WAN.
- Power on SG2: key switch to 2 o'clock, hold power 3 s until beep (head lights up). BG2: PSU switch right, key 12 o'clock, hold power 3 s.
- **Remote E-STOP**: red mushroom = stop; release = rotate clockwise then press **A**. On every power-on the robot is torque-off until **A** is pressed (beep = safety lock released). Other buttons do nothing.
- Optional ROBOTIS Hand (HX5-D20) install: rotate FRP42-A110K bracket 90°, enable terminating resistor slide switch inside hand back cover, 4-pin JST cable.
- Drawings: SG2 PDF/DWG/STEP download nos. 2215/2232/2228; BG2 2214/2231/2227; LG2 STEP 2229; workspace PDF 2230 (robotis.com/service/download.php?no=N).

---

## 3. Software architecture

- JetPack 6.2 host → Docker container `ai_worker` (Ubuntu 24.04, ROS 2 Jazzy). Image `robotis/ai-worker`.
- `ros2_control` at **100 Hz** (`read→update→write`), `DynamixelHardwareInterface` over Dynamixel SDK. Joints in position mode; base in velocity mode. Limits from URDF + YAML clamping; comm errors checked per motor.
- Controllers (follower):

| Controller | Joints | Command topic |
|---|---|---|
| `arm_l_controller` (JTC) | 7 arm + `gripper_l_joint1` (8) | `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory` |
| `arm_r_controller` (JTC) | 7 arm + `gripper_r_joint1` (8) | `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory` |
| `head_controller` (JTC) | 2 | `/leader/joystick_controller_left/joint_trajectory` |
| `lift_controller` (JTC) | 1 | `/leader/joystick_controller_right/joint_trajectory` |
| `joint_state_broadcaster` | all | publishes `/joint_states` |
| `swerve_drive_controller` (`ffw_swerve_drive_controller/SwerveDriveController`) | 6 | `/cmd_vel` |

  Last joint of each arm controller = gripper. Example config: `ffw_bringup/config/ffw_bg2_rev4_follower/ffw_bg2_follower_ai_hardware_controller.yaml`.
- Debug: `ros2 control list_controllers`, `ros2 topic echo /joint_states`, RViz2.
- Battery (SG2): `ros2 topic echo /ai_worker/battery/{left,right}/state --once` → `voltage`, `percentage` (0–1).
- Bringup logs (inside container): `/var/log/ai_worker_bringup/current`.
- Two core repos: `ai_worker` (bringup/control/teleop/nav) and `cyclo_intelligence` (IL workflow; replaces legacy `physical_ai_tools`). Also `cyclo_control` (QP motion controller), `cyclo_lab` (Isaac Lab), `robotis_applications` (`robotis_vuer` VR), `robotis_mujoco_menagerie` (MJCF).

### Docker (`ai_worker`)
```
git clone -b jazzy https://github.com/ROBOTIS-GIT/ai_worker.git
./docker/container.sh start|enter|stop|help
```
Volumes: `/dev`, X11, `./workspace:/workspace`, `../:/root/ros2_ws/src/ai_worker/`, `./lerobot/outputs:...physical_ai_tools/lerobot/outputs`, `./huggingface:/root/.cache/huggingface`. Anything outside mapped volumes is lost on container removal. Update = `git checkout jazzy && git pull` then `container.sh stop && start`.
User-PC prerequisites: Ubuntu, Docker Engine (+post-install), git, NVIDIA driver `nvidia-driver-570-server-open` (CUDA 12.8), NVIDIA Container Toolkit.

### Zenoh RMW (default since AI Worker 2.0.0)
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp`; router = `rmw_zenohd` (`ros2 run rmw_zenoh_cpp rmw_zenohd` or alias `zenohd`), port **7447**.
- **Zenoh daemon must be running before any bringup.** With Cyclo Manager installed it runs in the `zenoh_daemon` container (`docker ps | grep zenoh_daemon`) and persists across reboots — don't start a second one.
- External PC container: `export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;mode="client";connect/endpoints=["tcp/<robot_ip>:7447"]'`. Same host: `'transport/shared_memory/enabled=true'`.
- Keep `ROS_DOMAIN_ID` consistent (container default **30**). Check: `echo $RMW_IMPLEMENTATION; echo $ZENOH_CONFIG_OVERRIDE; ros2 node list; ros2 topic list`. If stuck: `ros2 daemon stop`, verify IP/port reachability.
- Switch to Fast DDS: edit `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in `~/.bashrc` of **every** ROS container (ai_worker, cyclo_intelligence, cyclo_manager…), then `docker restart ai_worker`.

---

## 4. Cyclo Manager (web ops panel on the Orin)

- Install on Orin: `pip install cyclo-manager; export PATH="$HOME/.local/bin:$PATH"; cyclo_manager up` (`--pull` for latest images). `cyclo_manager down`, `cyclo_manager update`. Auto-starts on boot after first `up`. Registers a host-agent systemd service; creates `cyclo_manager` (API, port **8081**) + `cyclo_manager_ui` (port **3000**), plus zenoh daemon and novnc-server containers (created, started on demand).
- URL: `http://ffw-SNPR48A1025.local:3000` (or IP). App page → Cyclo Manager dashboard or Cyclo Intelligence.
- **System page**: start Zenoh Daemon (green/red light); Robot Bringup ▶️ with type SG2/BG2/SH5/BH5/**Mobile** (base only, for Jog); LG2 leader; ⚙️ launch args (Start RViz, Use Simulation, Use Mock Hardware, Port Name, Launch Cameras/Lidar, Init Position, Init Position File); logs; Clear logs.
- **Pack position**: set Init Position File = `pack_position.yaml`, start bringup, stop after reaching pose.
- **Manage Containers**: start/stop/restart, logs, edit `.bashrc` (change `ROS_DOMAIN_ID` — must match across containers; restart Cyclo Manager afterwards), image cleanup/prune, browser Terminal into containers.
- **Version Update**: Package Update (stops/removes container, only `main`/`jazzy` branches, choose local-change strategy, **preserve `ffw_bringup/config/common/rs_serial.yaml`**, pull, recreate; image pull ~5 min), Agent Update (s6 agent inside containers must match manager version), Cyclo Manager Update (fallback: `cyclo-manager update` over SSH).
- **noVNC**: Start → reload → Dynamixel Wizard 2.0 icon. (Old standalone `docker-compose.novnc.yml`, port 8090, removed from ai_worker ≥1.2.2.)
- **Jog**: browser joystick for mobile base (works from phone on robot Wi-Fi); bring up `Mobile` type if only base is needed.

---

## 5. Teleoperation (LG2 leader)

- Leader back: power adapter → U2D2; U2D2 on/off recessed button; USB from U2D2 to a follower USB port. Wear via shoulder straps + chest/hip belts (v2: inner belt then outer Velcro belt).
- In `ai_worker` container (zenoh daemon running):
  - All-in-one: `ros2 launch ffw_bringup ffw_sg2_ai.launch.py` / alias `ffw_sg2_ai` (bg2 equivalents exist).
  - Separate: follower `ffw_sg2_follower_ai` (= `ffw_sg2_follower_ai.launch.py`, args `launch_cameras:=false`, `init_position:=false`, `start_rviz:=true`), leader `ffw_lg2_leader_ai`.
  - Inference: run **follower only**; never teleop + inference simultaneously.
- Safety behavior: follower does not move until **both hand triggers held >2 s**; then slow approach to leader pose, fast once close. Same gesture pauses (hold position). Requires ai_worker ≥1.1.10.
- Mapping: grip buttons → grippers; right joystick → lift; left joystick → head. SG2: press both switches → **SWERVE DRIVE MODE** (left stick = linear x/y, right stick Y = angular z); arms still move in swerve mode.
- Stop: neutral pose/pause, then Ctrl+C.
- Keyboard base teleop: `ros2 run ffw_teleop mobile_teleop` (W/S fwd/back, A/D turn, Space stop) or `python3 ~/ros2_ws/src/ai_worker/ffw_teleop/ffw_teleop/mobile_teleop.py`.

---

## 6. VR Teleoperation (Meta Quest 3 + ROBOTIS Vuer + Cyclo Control)

- Quest: enable Developer Mode (Meta Horizon app), display-off/sleep = 4 h, stationary boundary Large. Tape over proximity sensor to avoid sleep.
- Change default init pose to a "ready" pose (elbows 90°, hands forward) — arms must be in camera view (see Discord post).
- Server: `git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_applications.git`, `docker/container.sh start|enter`, `ros2 launch robotis_vuer vr.launch.py model:=sg2` (alias `vr model:=sg2`; `sh5` for hands). **Robot needs internet via WAN port** during VR.
- Headset browser: `https://<pc_ip>:8012?ws=wss://<pc_ip>:8012` (accept self-signed cert) → **Enter VR** at your operating spot (sets VR origin). Refresh + Enter VR again if server restarts.
- Robot side (`ai_worker` container): `ffw_sg2_follower_ai`, then `ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr` (alias `motion_controller controller_type:=vr`; add `hand:=true` for SH5).
- Gripper models: VR publishes only while **both squeeze buttons held** (deadman). Activate controller with **X + A** together; system checks pose delta, starts after 3 s, slow-start 5 s. Align arms to robot pose first. Release squeeze = pause.
- Hand models (SH5): toggle with one hand squeeze + other hand pinch held 3 s.
- Vuer version validated: **v0.1.5**. Troubleshoot: ROS_DOMAIN_ID (30), wired Ethernet to LAN port (USB-C adapter on Quest) if Wi-Fi laggy.

---

## 7. Cyclo Control (QP motion controller, repo `cyclo_control`)

Clone into `~/ros2_ws/src` inside the ai_worker container, build per README. Provides self-collision avoidance + joint limit/velocity constraints (not guaranteed — keep E-stop close).

Launch: `ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=<mode> [start_interactive_marker:=true] [disable_gripper_collisions:=true]`

| Mode | Input | Output |
|---|---|---|
| `movel` (default) | `robotis_interfaces/msg/MoveL` on `/r_goal_move`, `/l_goal_move` (pose in `base_link` + `time_from_start` = interpolation time); RViz markers `right_goal_marker`/`left_goal_marker` | arm JT topics above |
| `movej` | raw JTs on `/leader/joint_trajectory_command_broadcaster_{right,left}/raw_joint_trajectory` | filtered `.../joint_trajectory` |
| `bimanual_movel` | as movel + `/capture_grasp` (Bool) → then `/virtual_object_goal_move` | rigid two-hand constraint |
| `bimanual_movej` | as movej + `/capture_grasp` or auto via `gripper_grasp_threshold`/`gripper_grasp_hold_time` | |
| `vr`, `leader` | VR refs / leader URDF (`leader_urdf_path`, `arm`, `hand`, `reactivate_topic`) | |

- Using LG2 with movej: remap leader spawner outputs `joint_trajectory` → `raw_joint_trajectory` in `ffw_lg2_leader_ai.launch.py` (`--controller-ros-args -r ...`).
- Other topics: `/r_gripper_pose`, `/l_gripper_pose`, `/r_goal_pose`, `/l_goal_pose`, `/virtual_object_goal_pose`; lift via `/leader/joystick_controller_right/joint_trajectory` when `lift_vel_bound` ≠ 0.
- Config: `cyclo_motion_controller_ros/config/ai_worker_config.yaml` — shared (`control_frequency`, `time_step`, `trajectory_time`, `weight_damping`, `collision_buffer`, `collision_safe_distance`, `slack_penalty`, `cbf_alpha`, `joint_state_timeout`), MoveL (`kp_position`, `kp_orientation`, `weight_position/orientation`, `lift_vel_bound`…), MoveJ (`kp_joint`, `weight_tracking`, `command_timeout`…).
- Launch args: `config_file`, `follower_urdf_path`, `default_srdf_path`/`modified_srdf_path`, `base_frame` (base_link), `marker_scale` 0.2, `right/left_controlled_link` = `end_effector_{r,l}_link`.
- Example MoveL: position {x:0.35, y:∓0.20, z:0.85}, identity orientation, 2 s.

---

## 8. Navigation (Nav2, package `ffw_navigation`)

- Bring up follower (`ffw_sg2_follower_ai`), then in new terminal: SLAM mapping `ros2 launch ffw_navigation navigation.launch.py use_slam:=true`; drive via RViz `2D Goal Pose` or `mobile_teleop.py`.
- RViz configs: `~/ros2_ws/src/ai_worker/ffw_navigation/rviz/{mapping,navigation}.rviz` (run on user PC container with matching ROS_DOMAIN_ID). `/scan` needs Reliability **Best Effort**; map needs Durability **Transient Local**.
- Save map: `cd ~/ros2_ws/src/ai_worker/ffw_navigation/maps && ros2 run nav2_map_server map_saver_cli -f ./map` → `map.yaml`/`map.pgm`. Only a default Gazebo map ships.
- Navigate on saved map: `ros2 launch ffw_navigation navigation.launch.py`; set `2D Pose Estimate` (AMCL), then `2D Goal Pose`.
- Tuning: `config/amcl_localization.yaml`, `config/navigation.yaml`; MPPI variant `navigation_mppi.yaml` (set `params_file`).
- Tech story: default controller Regulated Pure Pursuit (0.5 m/s, lookahead 0.4 m); MPPI with `motion_model: "Omni"` gives holonomic crab motion; custom BT condition node `IsPathLengthUnder` switches `OmniController` (short path, PreferForward disabled) vs `CurveController` (long path, PathAlign w=14).
- **Easy Navi** (Cyclo Intelligence ≥1.2.0, AI Worker ≥2.0.2): browser Mapping/Navigation/Fix(map edit)/Goal/Initial/Save Map without RViz. WebGL — use Firefox if Chrome fails. Drive with Cyclo Manager Jog or keyboard teleop.

---

## 9. Imitation learning — Cyclo Intelligence (current, actively supported)

Components: **Cyclo Data** (record/review/edit/convert/HF ops) and **Cyclo Brain** (policy backend containers + inference runtime). Legacy **Physical AI Tools** is unmaintained (see §11).

### Setup
- Needs Docker ≥24 + Compose, git, 35 GB free, port **7080** free (`CYCLO_UI_PORT` to change), NVIDIA GPU only for inference.
- `curl -fsSL https://raw.githubusercontent.com/ROBOTIS-GIT/cyclo_intelligence/main/install.sh | bash` → `~/cyclo_intelligence`; update via `git pull && git submodule update --init --recursive`.
- `cd ~/cyclo_intelligence && ./docker/container.sh start && ./docker/container.sh enter`; set `ZENOH_CONFIG_OVERRIDE` in container `~/.bashrc` (same-machine vs client form, check with `grep` first); run `cyclo_intelligence` (= `ros2 launch orchestrator cyclo_intelligence_bringup.launch.py`); open `http://ffw-SNPR48A1025.local:7080/`.

### Data Recording
- Robot + camera nodes must already be running (teleop bringup). Home page: select robot type (only types with config under `cyclo_intelligence/shared/shared/robot_configs/<robot>_config.yaml`). Only topics listed in that config are recorded; add cameras under `observation.images.<name>` with `topic`, `msg_type: sensor_msgs/msg/CompressedImage`, optional `rotation_deg` (0/90/180/270).
- Record page: Camera Views, Topic Monitor (measured Hz), Rosbag Recorder panel (Task Num, Task Name, Task Instruction, Add License, Number of SubTasks + Sub Task Instructions). `Record Start` → `Save Episode` / `Discard Episode`. Leader buttons: left = cancel episode, right = start/save.
- Output: container `/workspace/rosbag2/<dataset>/<episode>/` = host `~/cyclo_intelligence/docker/workspace/rosbag2/...` containing `metadata.yaml`, `episode_info.json`, `*.mcap`, `videos/`, `camera_info/`.

### Data Tools (in order)
1. **Review Episodes** (replay, cameras, 3D, joints) → note bad indices.
2. **Delete Episodes** (comma list, "Compact indices after delete").
3. **Merge Rosbag Datasets** (≥2 sources → `/workspace/rosbag2`).
4. **Convert Dataset** → LeRobot v2.1 and/or v3.0 at chosen FPS (use camera's real rate, e.g. 15; slowest camera if mixed) → `/workspace/lerobot`.
5. **Hugging Face Upload/Download** — upload datasets; download trained models into `/workspace/model`.

### Model Training (done OFF the robot, your own env)
Supported (must have inference backend): ACT `act`, Diffusion `diffusion` (from scratch, single consumer GPU OK), SmolVLA (`lerobot/smolvla_base`), XVLA (`lerobot/xvla-base`), Pi0 (`lerobot/pi0_base`), Pi0.5 (`lerobot/pi05_base`), GR00T N1.7 (`nvidia/GR00T-N1.7-3B`). Only VLAs take a language instruction. Unsure → ACT.
- `git clone https://github.com/huggingface/lerobot.git && pip install -e '.[training]'`; `hf auth login; HF_USER=$(hf auth whoami | head -n 1)`.
- Entry point `lerobot-train --dataset.repo_id=${HF_USER}/my_dataset --policy.type=act --output_dir=outputs/train/act_run --job_name=act_run --policy.device=cuda --wandb.enable=true`.
- SmolVLA: use `--policy.type=smolvla --policy.pretrained_path=lerobot/smolvla_base` (NOT `--policy.path`, which fixes state dim at 6; AI Worker needs **16**), defaults freeze vision + `train_expert_only`.
- XVLA: `--policy.path=lerobot/xvla-base`, `--rename_map` for camera keys, `--policy.dtype=bfloat16`, `--policy.action_mode=auto`.
- Pi0/Pi0.5: `--policy.pretrained_path`, `--policy.gradient_checkpointing=true`; full fine-tune won't fit 24 GB → `--policy.train_expert_only=true`; needs gated `google/paligemma-3b-pt-224` access.
- GR00T: `--policy.type=groot --policy.base_model_path=nvidia/GR00T-N1.7-3B --policy.embodiment_tag=new_embodiment --policy.tune_llm=false --policy.tune_visual=false --policy.tune_projector=true --policy.tune_diffusion_model=true --policy.chunk_size=16 --policy.n_action_steps=16 --policy.use_bf16=true --dataset.image_transforms.enable=true`.
- Resume: `lerobot-train --config_path=outputs/train/<run>/checkpoints/last/pretrained_model/train_config.json --resume=true`. Multi-GPU via torchrun/accelerate.
- Push checkpoint to HF, then Data Tools → download Model into `/workspace/model`. Paths: LeRobot `/workspace/model/lerobot/<run>/checkpoints/<step>/pretrained_model`; GR00T `/workspace/model/groot/<run>/checkpoint-<step>`.

### Model Inference (Cyclo Brain)
- Stop teleop/leader; run follower-only bringup first.
- Select model → backend Docker image (`Pull` if "Image missing"; prebuilt ACT/GR00T images only for Blackwell amd64/arm64). Backend buttons ON/Restart/OFF; statuses `Policy Docker is off`, `Not created`, `Warming up`, `Main Down`, `Engine Down`.
- Deploy target: **3D Sim Deploy** (one action chunk in sim, blocks robot) → then **Real Robot Deploy**.
- Action request `Async` (non-blocking, avoid if inference slow) vs `Sync`. Rates: `Inference Hz` (policy), `Control Hz` (command publish).
- LeRobot panel: Policy Path (+ Task Instruction for language policies). GR00T panel: Task Instruction, optional TensorRT.
- Architecture: host service call LOAD/START/PAUSE → Main Runtime → InferenceRequester → EngineWorker (separate container) → RobotClient reads obs, InferenceEngine returns action_list → ActionChunkProcessor → RobotClient publishes commands.

### Behavior Trees (Cyclo Intelligence orchestrator)
- BT Manager web editor + BT Runtime ROS node (ON/OFF buttons; node must be ON before Start). Catalog auto-generated from Python classes via `/bt/nodes/catalog` (no `TreeNodesModel` needed). XML = BehaviorTree.CPP v4 format; tag name = Python class name; constructor kwargs = ports.
- Code: `cyclo_intelligence/orchestrator/orchestrator/bt/{bt_node.py,bt_nodes_loader.py,node_registry.py,bt_core.py,blackboard.py,actions/,controls/,templates/,trees/}`; launch `bt_node.launch.py`.
- Controls: `Sequence`, `Loop max_iterations` (0 = forever). Actions: `SendCommand command=LOAD|RESUME|STOP|CLEAR` (params `model="groot:n17"`, `policy_path`, `task_instruction`, `inference_mode="robot"`, `inference_hz`, `control_hz`, `chunk_align_window_s`), `JointControl` (`enable_head/arms/lift`, `head_positions`, `left/right_positions` 8 values, `lift_position`, `duration`; waits on `/joint_states`), `Rotate angle_deg` (uses `/odom`, cmd_vel), `Wait duration`.
- Custom node: subclass `BaseAction` in `bt/actions/`, implement `tick()` returning `NodeStatus`; click refresh in BT Manager.

---

## 10. Simulation

- **Gazebo**: `ros2 launch ffw_bringup ffw_sg2_follower_ai_gazebo.launch.py` (bg2 too). RViz-only model: `ros2 launch ffw_description ffw_sg2_rev1.launch.py` / `ffw_bg2_rev4.launch.py`. Hardware + RViz: follower launch with `start_rviz:=true`.
- **MoveIt**: `ros2 launch ffw_moveit_config moveit.launch.py [use_sim:=true]`.
- **Nav2 in Gazebo**: `ros2 launch ffw_navigation navigation.launch.py use_slam:=true use_sim_time:=true`; saved map: `... navigation.launch.py use_sim_time:=true`.
- **MuJoCo**: MJCF in `robotis_mujoco_menagerie/robotis_ffw`. **USD**: `cyclo_lab/source/cyclo_lab/data/robots/FFW`.
- **Cyclo Lab** (Isaac Lab ≥2.2; Docker ships Isaac Sim 5.1.0, Isaac Lab 2.3.0, CycloneDDS 0.10.2, robotis_dds_python, LeRobot 0.3.3 in `~/lerobot_env`): `git clone --recurse-submodules https://github.com/ROBOTIS-GIT/cyclo_lab.git; ./docker/container.sh start|enter|stop|logs|clean`.
  - RL: `python scripts/reinforcement_learning/rsl_rl/train.py --task Cyclo-Reach-FFW-BG2-v0 --num_envs=512 --headless`; play with `Cyclo-Reach-FFW-BG2-Play-v0`.
  - IL sim2real (FFW_SG2 pick-place): `record_demos.py` → `action_data_converter.py --action_type ik` → `annotate_demos.py` → `generate_dataset.py` → `action_data_converter.py --action_type joint` → `lerobot-python .../isaaclab2lerobot.py` → train/infer with physical_ai_tools → `inference_demos.py`.
  - SH5 VR-in-Isaac bridge: `python scripts/sim2real/bringup/sh5_dds_bringup.py --domain_id 30 --enable_gravity --enable_camera_views [--enable_environment]` (DDS domain must match ROS_DOMAIN_ID); subscribes arm/hand/lift JT topics + `/cmd_vel`, publishes `/joint_states`, `/tf`.

---

## 11. Legacy: Physical AI Tools (unmaintained, still installed here)

- Containers `physical_ai_server` / `physical_ai_manager`; repo `~/physical_ai_tools`; `./docker/container.sh enter`; server: `ros2 launch physical_ai_server physical_ai_server_bringup.launch.py` (alias `ai_server`); Web UI at `http://ffw-<serial>.local`.
- Recording modes: Single Task (fixed episodes/time, reset time) vs Multi-Task (scenarios). Fields: Task Name, Instruction(s), Push to Hub (needs wired internet; else `huggingface-cli upload <user>/<repo> <path> / --repo-type=dataset`), Private, User ID (HF token read+write), FPS (15), Tags, Warmup/Episode/Reset Time, Num Episodes, Optimized Save, Record Rosbag2 (~9× storage, to `~/physical_ai_tools/docker/workspace/rosbag2/<user>/`). Leader buttons: left = retry, right = next.
- Datasets: host `~/physical_ai_tools/docker/huggingface/lerobot/<HF_USER>/<dataset>/{data,meta,videos}` = container `/root/.cache/huggingface/lerobot/`. Schema: `action` (leader), `observation.state` (follower), `observation.images.cam_head/cam_wrist_1/cam_wrist_2`, `timestamp`, `frame_index`, `episode_index`, `index`, `task_index`.
- Visualize: `python lerobot/scripts/visualize_dataset_html.py --host 0.0.0.0 --port 9091 --repo-id <id>` from `/root/ros2_ws/src/physical_ai_tools/lerobot/src`.
- Training (Web UI Training page or `python -m lerobot.scripts.train --dataset.repo_id=... --policy.type=act --output_dir=outputs/train/... --policy.device=cuda`); outputs `~/physical_ai_tools/lerobot/outputs/train/` (= container `/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/`). Inference page: Policy Path = `.../pretrained_model/`, FPS = recording FPS.

---

## 12. Troubleshooting: DYNAMIXEL multi-turn error

Log: `[ID:0xx] ... Error Code Details: 0xc (Multi-turn Error)` then `Rebooting...`.
1. E-stop, physically align slit marks to home for the faulted joint (lift: slider top 9 cm below rail top; wheels straight), release E-stop.
2. Dynamixel Wizard 2.0 via Cyclo Manager noVNC (`:3000` → AI worker → noVNC; latest wizard v2.12.2.2, refresh with `cyclo_manager down && cyclo_manager up --pull`).
3. Scan: Protocol 2.0, port `/dev/ttyUSB4` (LG2 leader: ttyUSB4–6), 4,000,000 bps.
4. Select the motor; DYNAMIXEL-Y latest firmware **13** → Tools → Firmware Update All; then Factory Reset; confirm Return Delay Time non-zero. **Lift (ID 81): never press Release/Engage Brake.**
5. Tools → Encoder Battery Change (slits still aligned).
6. Verify position within ±1.5°; else realign and repeat.
7. Post-fix: IDs **1, 3, 31, 33** → Homing Offset = **32768** (22.5°). ID 81 → Max Position Limit(76)=0, Min Position Limit(84)=−50000, Gear Ratio Numerator(96)=131072, Denominator(100)=125.
8. Re-run bringup.

---

## 13. Technical stories (reference designs)

- **Isaac CenterPose pick & place** (branch `feature-centerpose`): ZED → Isaac ROS CenterPose on GPU PC (`/centerpose/detections`, Detection3DArray) → project detection to pixel, sample real depth from `/zedm/zed_node/depth/depth_registered` (ZED depth mode ULTRA in `ffw_bringup/config/common/common_stereo.yaml`), back-project, TF to `base_link`, fixed grasp z, quaternion → gripper roll/yaw via calibrated linear fit + 180° flip heuristic → MoveL via cyclo_motion_controller_ros. Wired USB 3.0 to user PC required.
- **GR00T N1.5 sorting** (CoRL 2025): 800 episodes/10 h, 8× B200 for ~20 h, ~85 % over 100 trials; Orin runs ai_worker + physical_ai_tools containers, RTX 5090 workstation runs GR00T via ZMQ; 40 ms inference, 10 FPS control. Failures: color misclassification 50 %, grasp 30 %, phantom placement 20 %.
- **Nav2** and **SH5 Isaac VR** covered in §8 / §10.

---

## 14. Support & resources

- Issues: https://github.com/ROBOTIS-GIT/ai_worker/issues · Discord https://discord.gg/robotis (FAQ + release notes channels) · sensitive: ost@robotis.com · sales contactus2@robotis.com / america@robotis.com.
- Docker Hub `robotis/ai-worker`; YouTube @ROBOTISOpenSourceTeam.
- Licenses: ROBOTIS repos Apache 2.0; check LICENSE per model/asset.

---

## 15. RViz2 from inside the `ai_worker` container (X11 over SSH)

The Orin's own X server (`:0`, GDM) is the HDMI head, not your laptop. Two ways to see RViz remotely:

### Option A (in use): `ssh -X` forwarding (per-session cookie) — see below.

### Option B (NOT applied — known black-screen issue): noVNC display `:99`
The `novnc-server` container (Cyclo Manager) runs Xvfb `:99` (1920×1080, TCP 6099, `-ac`, no cookie). `export DISPLAY=:99; unset XAUTHORITY; rviz2` from `ai_worker` opens a window viewable at `http://ffw-SNPR48A1025.local:8090` / Cyclo Manager → noVNC, **but the 3D panel is black**.

Root cause (diagnosed 2026-08-23 with screenshots + `glxgears`): Mesa llvmpipe uploads frames via MIT-SHM (`X_ShmPutImage BadValue`); `ai_worker` is `ipc: host` while `novnc-server` is `ipc: private`, so Xvfb can't attach the segments. Qt panels survive because `QT_X11_NO_MITSHM=1` works for Qt; Mesa has no equivalent switch. Not fixed by GL vendor forcing, `LIBGL_DRI3_DISABLE`, indirect GLX, or non-loopback DISPLAY.

Fix if ever wanted (reversible, no drivers touched): add `ipc: host` to the `novnc-server` service in
`~/.local/lib/python3.10/site-packages/cyclo_manager_cli/docker/docker-compose.yml` (next to `network_mode: host`), then recreate only that container. Caveats: `cyclo_manager update`/reinstall overwrites the file; noVNC desktop restarts briefly. Worth proposing upstream to ROBOTIS as the general fix for "black RViz in noVNC".

### Option A details: `ssh -X` forwarding
Connect with `ssh -X` and forward the cookie into the container. The display number changes every SSH session, and the forwarding channel dies when that SSH session ends.

**Host (as `robotis`, in the `ssh -X` session):**
```bash
XAUTH=/tmp/.xauth.rviz
rm -f $XAUTH && touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 644 $XAUTH
docker cp $XAUTH ai_worker:$XAUTH
echo $DISPLAY          # e.g. localhost:10.0 — note <N>
```

**Inside the container** (`cd ~/ai_worker && ./docker/container.sh enter`; type after the prompt appears):
```bash
export DISPLAY=localhost:<N>.0
export XAUTHORITY=/tmp/.xauth.rviz
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
rviz2                                   # or any launch with start_rviz:=true
```

Notes:
- Any launch that spawns RViz (`ffw_sg2_follower_ai.launch.py start_rviz:=true`, nav `rviz2 -d …/ffw_navigation/rviz/*.rviz`) must be run from a shell where both exports are set.
- The follower RViz config is `ffw_description/rviz/ffw_sg2.rviz`.
- Claude (running on the host) can perform the host-side cookie steps if it knows `<N>`: it finds it from your live shell's env (`/proc/<pid>/environ` of your `bash` under `sshd: robotis@pts/*`). Claude cannot see the RViz window — report what you observe.
- Dry-run of the arm stack without motors: `ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py use_mock_hardware:=true launch_cameras:=false launch_lidar:=false start_rviz:=true` — mock hardware mirrors commands into `/joint_states`, so RViz shows exactly what would be sent to the motors.

## 16. Local setup: cyclo_control `movej` teleop + data collection (this robot, 2026-08-24)

Verified on the real SG2. Leader → `ai_worker_movej_controller` (QP: joint limits, velocity limits, self-collision) → follower. Files live in `~/ai_worker/docker/workspace/` (= `/workspace` in the container, git-ignored):

| File | Purpose |
|---|---|
| `ffw_lg2_leader_movej.launch.py` | copy of `ffw_lg2_leader_ai.launch.py`; remaps the broadcaster's `…/joint_trajectory` → `…/raw_joint_trajectory` so movej is the only publisher on the follower topics; loads the params file below |
| `ffw_lg2_leader_movej_controllers.yaml` | copy of `ffw_lg2_leader_ai_hardware_controller.yaml` + `sync_threshold: 0.025` |

Run order (3 container terminals): `ffw_sg2_follower_ai` → `motion_controller controller_type:=movej` → `ros2 launch /workspace/ffw_lg2_leader_movej.launch.py`. Never `ffw_sg2_ai` / Cyclo Manager "LG2 leader" together with movej (double publisher). Check before triggering: `ros2 topic info -v /leader/joint_trajectory_command_broadcaster_right/joint_trajectory` → exactly one publisher, `ai_worker_movej_controller`.

### Why `sync_threshold: 0.025` (default 0.01)
- The leader broadcaster (`ffw_joint_trajectory_command_broadcaster`, `update()` ~L442–563) publishes each sample with `time_from_start` stretched 0–0.4 s proportional to the mean leader–follower error until that mean (16 arm+gripper joints) ≤ `sync_threshold`; then it sends `time_from_start = 0` permanently (until the next trigger toggle).
- movej (`ai_worker_movej_controller_node.cpp` L295) resets its running setpoint to the *measured* joint positions on every message with `time_from_start > 0`. While that happens the follower creeps toward the leader gated by motor response ("sluggish"); once `time_from_start = 0` movej integrates open-loop with `kp_joint = 50` (~20 ms) and feels reactive.
- The cyclo model clamps elbow `joint4 ≤ −0.70 rad` and shoulder-roll `joint2` at 0 (left as-is — likely intentional for singularity/torso clearance). A straight leader elbow therefore contributes permanently to the mean error and can block sync at 0.01.
- Measured with 0.025: sluggish phase 1–5 s after engage (best ≈1 s with a good pose match). Manipulation after the flip is unaffected. Next candidate if still slow: 0.04; keep ≤ ~0.08 (a single far-off joint could snap at the flip). Revert: set 0.01 or delete the line, relaunch the leader only.

### Operating notes
- Start pose under movej: arms down, **elbows −0.70 rad (bent ~40°)**; movej moves the elbows there by itself at start-up. Keep your elbows bent ≥ 40° while teleoperating — straighter is unreachable.
- Trigger (both hand triggers 2 s) is a **toggle**: press again = pause. `Auto mode ACTIVATED/STOPPED` and `Joints synced` lines in the leader log show the state.
- Recording: engage → wait until it feels reactive (sync flip) → then Record Start, so the slow phase isn't in the episode. Return the leader to the start pose before the next episode.
- Cyclo Intelligence records action from the *filtered* movej topics; `…/raw_joint_trajectory` (pre-filter leader intent) was added to `recording.extra_topics` in `~/cyclo_intelligence/shared/shared/robot_configs/ffw_sg2_rev1_config.yaml` (uncommitted).
- Collision model is coarse (capsules on arm links 2/4/7 only, chassis boxes up to z≈0.35 m) and the CBF is soft — not a substitute for the E-stop.

## 17. Quick command cheat-sheet (run on this Orin)

```bash
ssh robotis@ffw-SNPR48A1025.local            # pw: root
docker ps | grep zenoh_daemon                 # zenoh must be up
cd ~/ai_worker && ./docker/container.sh enter
ffw_sg2_ai                                    # leader+follower teleop
ffw_sg2_follower_ai                           # follower only (inference/nav/VR)
ffw_lg2_leader_ai                             # leader only
ros2 launch ffw_navigation navigation.launch.py use_slam:=true
motion_controller controller_type:=vr         # cyclo_control (VR)
ros2 topic echo /ai_worker/battery/left/state --once
# Web UIs: Cyclo Manager :3000 | Cyclo Intelligence :7080 | legacy Physical AI Manager :80 | Vuer :8012
```
