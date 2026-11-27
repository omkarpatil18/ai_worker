# Motor debug tools (FFW-SG2 follower)

Small standalone scripts written while diagnosing the right shoulder servo (DYNAMIXEL ID 1,
`arm_r_joint1`, YM080-230-R099), which repeatedly trips Overload (error 0x05). They are not ROS
packages; run them with `python3` inside the `ai_worker` container, where this repo is mounted at
`/root/ros2_ws/src/ai_worker`.

```bash
docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/<script>.py --help"
```

| Script | What it does | Touches the robot? |
|---|---|---|
| `motor_profile.py` | Records every `/joint_states` message (position, velocity, current for all joints) and `/ffw_follower/dxl_state` to CSV; prints a 30 s summary. | Listens only |
| `live_current.py` | Serves a live left-vs-right current plot (in amperes) at `http://<robot-ip>:8765`. | Listens only |
| `plot_current.py` | Draws a PNG of shoulder current from a `motor_profile` recording. | No (reads a CSV) |
| `arm_circles.py` | Small, slow mirrored shoulder circles around the tabletop home pose. | **Moves both arms** |
| `arm_updown.py` | Raises and lowers one or both shoulders (joint 1 only), with pauses at each end. | **Moves the arm(s)** |
| `dxl_readonly_compare.py` | Reads control-table registers straight from the servos (READ instruction only) and compares right vs left. | Bus read; **bringup must be stopped** |
| `overload_watchdog.py` | Watches `/rosout` for servo fault messages. Dry run by default; `--arm` switches all follower torque off on the first fault. | Experimental, see below |

## Things learned the hard way

- **Units.** `/joint_states` effort for the YM070/YM080 arm servos is the raw Present Current times
  the factor in the driver's model file (0.0769 for YM080, 0.04 for YM070); the raw unit is 0.01 A.
  So 1 effort unit = 0.13 A on joints 1-3 and 0.25 A on joints 4-6. The driver's "N m" label is
  misleading. YM080-230-R099: continuous 10.7 A, maximum 22.4 A.
- **Do not poll `/ffw_follower/get_dxl_data` while the robot runs.** Each call blocked the
  follower's 100 Hz read/write loop for up to 1 s (seen 2026-11-26). `motor_profile.py` keeps its
  temperature polling off by default for that reason. Read registers with
  `dxl_readonly_compare.py` with the bringup stopped instead.
- **Starting these Python tools can cost one skipped control cycle** (CPU load at launch). Start
  them while the arms are at rest.
- **A latched (Overload) shoulder free-falls.** Recorded 2026-11-27: 0.41 rad in 0.33 s, landing on
  the table, with no dynamic braking visible. A watchdog that reacts to the error message is too
  late for that joint, which is why `overload_watchdog.py` stays experimental. A watchdog on
  current (seconds of warning before the trip) would be the useful version.
- **Motion scripts** refuse to start unless the arms are within 0.05 rad of the tabletop home pose
  and nothing else is publishing on the arm command topics; they stop publishing if a joint drifts
  more than 0.15 rad from its command. Their paths were checked with FK for the 29.25 in table
  setup only. Re-check before using them in a different setup.

Recordings go to `/workspace/motor_profile_<date_time>/` in the container
(`~/ai_worker/docker/workspace/` on the host, git-ignored).
