#!/usr/bin/env python3
"""Small, slow, mirrored circles with both arms around the tabletop home pose.

Purpose: load both shoulders the same way so their currents can be compared
(run motor_profile.py at the same time).

Motion (joint space, both arms mirrored):
    j1 (shoulder) = home_j1 + A(t) * sin(w t)
    j2 (shoulder) = home_j2 +/- A(t) * (1 - cos(w t))      (+ left, - right: away from the body)
  every other joint and both grippers hold their current measured position.
  Defaults: amplitude 0.07 rad (hand loop about 7 cm across), period 7 s, 6 loops (42 s),
  so joint speed stays under about 0.065 rad/s (FK-checked 2026-11-26: fingertips >= 12.7 cm
  above the table, hands 0.52-0.58 m in front of the base, arms never closer than at home).
  A(t) ramps in over the first loop and
  out over the last, so the motion starts and ends at the home pose at zero speed.
  Using (1 - cos) keeps j2 at or outside home, i.e. the hands never move toward each other.

Safety checks:
  before moving  - every arm joint must be within --start-tol (0.05 rad) of the home pose
                 - nobody else may be publishing on the two arm command topics (leader, bag play)
  while moving   - stops publishing (servos hold their last target) if any commanded joint is
                   more than --max-err (0.15 rad) away from its measured position, or if
                   /joint_states goes quiet for 0.5 s
  Ctrl+C         - first press: ramps back to home over 3 s; second press: stops at once.

Run inside the ai_worker container with the follower bringup up and no leader/teleop running:
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/arm_circles.py --dry-run"   # checks only
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/arm_circles.py"
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HOME = {  # /workspace/poses/tabletop_manip/initial_state.json in cyclo_intelligence (same values)
    "arm_l_joint1": -1.5994, "arm_l_joint2": 0.4105, "arm_l_joint3": -2.3, "arm_l_joint4": -1.0982,
    "arm_l_joint5": 2.0193, "arm_l_joint6": 0.7225, "arm_l_joint7": 0.1107,
    "arm_r_joint1": -1.5994, "arm_r_joint2": -0.4105, "arm_r_joint3": 2.3, "arm_r_joint4": -1.0982,
    "arm_r_joint5": -2.0193, "arm_r_joint6": 0.7225, "arm_r_joint7": -0.1107,
}
SIDES = {
    "left": ("/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
             [f"arm_l_joint{i}" for i in range(1, 8)] + ["gripper_l_joint1"], +1.0),
    "right": ("/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
              [f"arm_r_joint{i}" for i in range(1, 8)] + ["gripper_r_joint1"], -1.0),
}
RATE_HZ = 100.0


class ArmCircles(Node):
    def __init__(self, a):
        super().__init__("arm_circles")
        self.a = a
        self.meas = {}
        self.meas_t = 0.0
        self.create_subscription(JointState, "/joint_states", self.on_js, qos_profile_sensor_data)
        self.pubs = {s: self.create_publisher(JointTrajectory, topic, 10)
                     for s, (topic, _, _) in SIDES.items()}

    def on_js(self, msg):
        self.meas.update(zip(msg.name, msg.position))
        self.meas_t = time.monotonic()

    def other_publishers(self):
        out = []
        for s, (topic, _, _) in SIDES.items():
            for info in self.get_publishers_info_by_topic(topic):
                if info.node_name != self.get_name():
                    out.append(f"{topic} <- {info.node_namespace}/{info.node_name}")
        return out

    def target(self, t, amp):
        w = 2 * math.pi / self.a.period
        q = {}
        for _, (_, joints, sign) in SIDES.items():
            for j in joints:
                q[j] = self.hold[j]
            side = joints[0][:5]  # "arm_l" / "arm_r"
            q[f"{side}_joint1"] = HOME[f"{side}_joint1"] + amp * math.sin(w * t)
            q[f"{side}_joint2"] = HOME[f"{side}_joint2"] + sign * amp * (1 - math.cos(w * t))
        return q

    def publish(self, q):
        for s, (_, joints, _) in SIDES.items():
            msg = JointTrajectory()
            msg.joint_names = joints
            p = JointTrajectoryPoint()
            p.positions = [float(q[j]) for j in joints]
            msg.points = [p]
            self.pubs[s].publish(msg)


def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * x)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--amp", type=float, default=0.07, help="rad on j1 and j2 (max 0.10)")
    ap.add_argument("--period", type=float, default=7.0, help="seconds per loop (min 6)")
    ap.add_argument("--loops", type=int, default=6)
    ap.add_argument("--start-tol", type=float, default=0.05)
    ap.add_argument("--max-err", type=float, default=0.15)
    ap.add_argument("--dry-run", action="store_true", help="run the checks and print the plan, do not move")
    a = ap.parse_args()
    if not (0 < a.amp <= 0.10) or a.period < 6.0 or a.loops < 2:
        raise SystemExit("refusing: amp must be in (0, 0.10] rad, period >= 6 s, loops >= 2")

    # Keep ROS alive on Ctrl+C so the return-to-home ramp can still publish.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = ArmCircles(a)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0 and len(n.meas) < 16:
        rclpy.spin_once(n, timeout_sec=0.1)
    try:
        missing = [j for _, (_, js, _) in SIDES.items() for j in js if j not in n.meas]
        if missing:
            raise SystemExit(f"no /joint_states for {missing}; is the follower bringup up?")
        off = {j: n.meas[j] - h for j, h in HOME.items()}
        bad = {j: round(v, 3) for j, v in off.items() if abs(v) > a.start_tol}
        print("start offsets from home (rad):", {j: round(v, 3) for j, v in off.items()})
        if bad:
            raise SystemExit(f"refusing: joints not at home within {a.start_tol} rad: {bad}")
        others = n.other_publishers()
        if others:
            raise SystemExit("refusing: something else commands the arms: " + "; ".join(others))
        n.hold = {j: n.meas[j] for _, (_, js, _) in SIDES.items() for j in js}
        vmax = a.amp * 2 * math.pi / a.period
        dur = a.loops * a.period
        print(f"plan: both arms, j1 +/-{a.amp} rad, j2 0..{2*a.amp} rad outward, period {a.period} s, "
              f"{a.loops} loops ({dur:.0f} s), peak joint speed {vmax:.3f} rad/s; grippers held at "
              f"{n.hold['gripper_l_joint1']:.3f} / {n.hold['gripper_r_joint1']:.3f}")
        if a.dry_run:
            print("dry run: checks passed, nothing published")
            return

        print("moving... Ctrl+C once = ramp back to home (3 s), twice = stop at once")
        start = time.monotonic()
        stop_at = None
        last_q = None
        amp = amp_at_stop = 0.0
        while True:
            try:
                now = time.monotonic()
                t = now - start
                if stop_at is None and t >= dur:
                    break
                amp = a.amp * smooth(t / a.period) * smooth((dur - t) / a.period)
                if stop_at is not None:
                    amp_stop = amp_at_stop * (1 - smooth((now - stop_at) / 3.0))
                    amp = amp_stop
                    if now - stop_at >= 3.0:
                        break
                q = n.target(t, amp)
                rclpy.spin_once(n, timeout_sec=0.0)
                if now - n.meas_t > 0.5:
                    print("STOP: /joint_states silent for 0.5 s; holding last target")
                    return
                err = {j: abs(q[j] - n.meas[j]) for j in HOME}
                worst = max(err, key=err.get)
                if err[worst] > a.max_err:
                    print(f"STOP: {worst} is {err[worst]:.3f} rad from its command; holding last target")
                    return
                n.publish(q)
                last_q = q
                time.sleep(max(0.0, 1.0 / RATE_HZ - (time.monotonic() - now)))
            except KeyboardInterrupt:
                if stop_at is not None:
                    print("stopped at once; servos hold the last target")
                    return
                stop_at = time.monotonic()
                amp_at_stop = amp
                print("Ctrl+C: returning to home over 3 s (Ctrl+C again to stop at once)")
        if last_q is not None:
            n.publish(n.target(0.0, 0.0))  # exact home for j1/j2, everything else held
        print("done: back at home")
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
