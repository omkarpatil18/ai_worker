#!/usr/bin/env python3
"""Raise and lower one shoulder (joint 1) only; every other joint holds still.

Purpose: load one shoulder servo alone (one-armed branch default: the LEFT shoulder) with a clean
lift / lower cycle, to watch its current (run motor_profile.py / live_current.py alongside).

Motion (ONE-ARMED BRANCH default: left shoulder only; --side both or right for the others), per cycle:
  smooth rise by A over period/2 -> hold at the top for --pause -> smooth descent over period/2
  -> hold at home for --pause. The holds make it easy to line the motion up with the current plot.
  j1 only ever moves UP from home (negative direction) and back, so the hand never gets closer to
  the table than at home. All other joints and the grippers hold their measured positions.
  Moving one joint swings the hand on an arc about the shoulder (not a straight line).
  Defaults: A = 0.20 rad, period 8 s (4 s up, 4 s down), pause 3 s, 6 cycles (84 s),
  peak speed 0.08 rad/s.
  FK-checked 2026-11-26 for A up to 0.30 rad on the right arm: at 0.20 rad the fingertip rises
  12 cm and moves 8.5 cm forward (x 0.55 -> 0.64 m); arm stays >= 30 cm from the head joints and
  the gap to the other arm only grows. The first and last cycle are eased in/out.

Safety checks (same as arm_circles.py):
  before moving  - the arm(s) to move must be within --start-tol (0.05 rad) of home; the other
                   arm may be anywhere only with --other-arm-anywhere (no pose is assumed for
                   it, and clearance to it is then your visual check); nobody else publishing
                   on the arm command topics
  while moving   - stops publishing (servos hold the last target) if any commanded joint is more
                   than --max-err (0.15 rad) from its measured position, or /joint_states is
                   silent for 0.5 s
  Ctrl+C         - once: ease back to home over 3 s; twice: stop at once.

Run inside the ai_worker container, bringup up, no leader/teleop/replay running:
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/arm_updown.py --dry-run"
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/arm_updown.py"                 # left shoulder (default)
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/arm_updown.py --side right"    # right (ID 1): needs the both-arms bringup, powers the faulty servo
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/arm_updown.py --side both"     # both shoulders
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HOME = {
    "arm_l_joint1": -1.5994, "arm_l_joint2": 0.4105, "arm_l_joint3": -2.3, "arm_l_joint4": -1.0982,
    "arm_l_joint5": 2.0193, "arm_l_joint6": 0.7225, "arm_l_joint7": 0.1107,
    "arm_r_joint1": -1.5994, "arm_r_joint2": -0.4105, "arm_r_joint3": 2.3, "arm_r_joint4": -1.0982,
    "arm_r_joint5": -2.0193, "arm_r_joint6": 0.7225, "arm_r_joint7": -0.1107,
}
SIDES = {
    "left": ("/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
             [f"arm_l_joint{i}" for i in range(1, 8)] + ["gripper_l_joint1"]),
    "right": ("/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
              [f"arm_r_joint{i}" for i in range(1, 8)] + ["gripper_r_joint1"]),
}
RATE_HZ = 100.0


def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * x)


def profile(tc, move, pause):
    """Fraction of the lift (0 = home, 1 = top) at time tc within one cycle:
    smooth rise over `move`, hold at the top for `pause`, smooth descent over `move`, hold at home."""
    if tc < move:
        return smooth(tc / move)
    if tc < move + pause:
        return 1.0
    if tc < 2 * move + pause:
        return 1.0 - smooth((tc - move - pause) / move)
    return 0.0


class UpDown(Node):
    def __init__(self, sides):
        super().__init__("arm_updown")
        self.sides = sides
        self.meas = {}
        self.meas_t = 0.0
        self.create_subscription(JointState, "/joint_states", self.on_js, qos_profile_sensor_data)
        # only the arm(s) being moved get a publisher; the other arm is not commanded at all
        self.pubs = {s: self.create_publisher(JointTrajectory, SIDES[s][0], 10) for s in sides}

    def on_js(self, msg):
        self.meas.update(zip(msg.name, msg.position))
        self.meas_t = time.monotonic()

    def other_publishers(self):
        out = []
        for s in self.sides:
            for info in self.get_publishers_info_by_topic(SIDES[s][0]):
                if info.node_name != self.get_name():
                    out.append(f"{SIDES[s][0]} <- {info.node_namespace}/{info.node_name}")
        return out

    def target(self, offset):
        q = dict(self.hold)
        for s in self.sides:
            j1 = SIDES[s][1][0]
            q[j1] = HOME[j1] - offset          # negative j1 = arm up
        return q

    def publish(self, q):
        for s in self.sides:
            msg = JointTrajectory()
            msg.joint_names = SIDES[s][1]
            p = JointTrajectoryPoint()
            p.positions = [float(q[j]) for j in SIDES[s][1]]
            msg.points = [p]
            self.pubs[s].publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", choices=["right", "left", "both"], default="left")
    ap.add_argument("--amp", type=float, default=0.20, help="rad the shoulder is raised from home (max 0.30)")
    ap.add_argument("--period", type=float, default=8.0, help="seconds of motion per cycle: half up, half down (min 6)")
    ap.add_argument("--pause", type=float, default=3.0, help="seconds held at the top and again at home (0-30)")
    ap.add_argument("--loops", type=int, default=6)
    ap.add_argument("--start-tol", type=float, default=0.05)
    ap.add_argument("--max-err", type=float, default=0.15)
    ap.add_argument("--other-arm-anywhere", action="store_true",
                    help="allow the arm that is not moved to be away from home (you checked clearance by eye)")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if not (0 < a.amp <= 0.30) or a.period < 6.0 or a.loops < 1 or not (0 <= a.pause <= 30):
        raise SystemExit("refusing: amp must be in (0, 0.30] rad, period >= 6 s, loops >= 1, pause 0-30 s")
    sides = ["left", "right"] if a.side == "both" else [a.side]

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # keep ROS alive for the Ctrl+C ramp
    n = UpDown(sides)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0 and len(n.meas) < 16:
        rclpy.spin_once(n, timeout_sec=0.1)
    try:
        cmd_joints = [j for s in sides for j in SIDES[s][1]]
        missing = [j for j in list(HOME) + cmd_joints if j not in n.meas]
        if missing:
            raise SystemExit(f"no /joint_states for {missing}; is the follower bringup up?")
        off = {j: n.meas[j] - h for j, h in HOME.items()}
        moved = {j for s in sides for j in SIDES[s][1]}
        bad = {j: round(v, 3) for j, v in off.items() if abs(v) > a.start_tol and j in moved}
        if bad:
            raise SystemExit(f"refusing: joints of the arm to move are not at home within {a.start_tol} rad: {bad}")
        # The arm that is NOT moved may be anywhere (e.g. unpowered in one-armed operation). No pose
        # is assumed for it, but the clearance between the arms was only FK-checked with it at home.
        other = {j: round(v, 3) for j, v in off.items() if abs(v) > a.start_tol and j not in moved}
        if other and not a.other_arm_anywhere:
            raise SystemExit(
                "refusing: the other arm is not at its home pose "
                f"({other}). Clearance between the arms was only checked with it at home. Look at "
                "the robot, make sure the moving arm's path (up to 0.3 rad up from home) is clear of "
                "it, then rerun with --other-arm-anywhere.")
        if other:
            print("note: the other arm is away from home; clearance to it is NOT checked by this script")
        # A side with no subscriber on its command topic has no controller behind it (e.g. the
        # right arm in one-armed operation): commands would go nowhere, so refuse up front.
        dead = [s for s in sides if n.count_subscribers(SIDES[s][0]) == 0]
        if dead:
            raise SystemExit(
                f"refusing: no controller is listening for the {' and '.join(dead)} arm "
                f"({', '.join(SIDES[s][0] for s in dead)}). In one-armed operation only the left "
                "arm can be moved.")
        others = n.other_publishers()
        if others:
            raise SystemExit("refusing: something else commands the arm(s): " + "; ".join(others))
        n.hold = {j: n.meas[j] for j in cmd_joints}
        move = a.period / 2
        cycle = a.period + 2 * a.pause
        dur = a.loops * cycle
        print(f"plan: {a.side} shoulder(s) (j1) only: up {a.amp} rad in {move:.0f} s, pause {a.pause:.0f} s at the top, "
              f"down in {move:.0f} s, pause {a.pause:.0f} s at home; {a.loops} cycles ({dur:.0f} s), "
              f"peak speed {a.amp * math.pi / (2 * move):.3f} rad/s; all other joints hold")
        if a.dry_run:
            print("dry run: checks passed, nothing published")
            return

        print("moving... Ctrl+C once = ease back to home (3 s), twice = stop at once")
        start = time.monotonic()
        stop_at, stop_from, offset = None, 0.0, 0.0
        while True:
            try:
                now = time.monotonic()
                t = now - start
                if stop_at is None:
                    if t >= dur:
                        break
                    offset = a.amp * profile(t % cycle, move, a.pause)
                else:
                    offset = stop_from * (1 - smooth((now - stop_at) / 3.0))
                    if now - stop_at >= 3.0:
                        break
                q = n.target(offset)
                rclpy.spin_once(n, timeout_sec=0.0)
                if now - n.meas_t > 0.5:
                    print("STOP: /joint_states silent for 0.5 s; holding last target")
                    return
                err = {j: abs(q[j] - n.meas[j]) for j in cmd_joints}
                worst = max(err, key=err.get)
                if err[worst] > a.max_err:
                    print(f"STOP: {worst} is {err[worst]:.3f} rad from its command; holding last target")
                    return
                n.publish(q)
                time.sleep(max(0.0, 1.0 / RATE_HZ - (time.monotonic() - now)))
            except KeyboardInterrupt:
                if stop_at is not None:
                    print("stopped at once; servos hold the last target")
                    return
                stop_at, stop_from = time.monotonic(), offset
                print("Ctrl+C: easing back to home over 3 s (Ctrl+C again to stop at once)")
        n.publish(n.target(0.0))
        print("done: back at home")
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
