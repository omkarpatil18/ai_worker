#!/usr/bin/env python3
"""Read-only motor profiler for the FFW-SG2 follower.

Records, until Ctrl+C:
  joint_states.csv  every /joint_states message: position, velocity, effort (= servo Present
                    Current, raw driver units) for all joints
  dxl_state.csv     /ffw_follower/dxl_state once per second and on every change
                    (comm state, torque on/off and hardware error per servo ID)
  temps.csv         Present Motor/Inverter Temperature and Error Code of the chosen IDs, read with
                    /ffw_follower/get_dxl_data every --poll seconds (OFF by default: each read stalls the driver loop)
and prints a one-line summary every 30 s.

It only subscribes and calls the read service: it never commands, enables torque or writes a
register. Run inside the ai_worker container, after the follower bringup is up:

    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/motor_profile.py"

Output goes to /workspace/motor_profile_<date_time>/ (host: ~/ai_worker/docker/workspace/).
"""
import argparse
import collections
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from dynamixel_interfaces.msg import DynamixelState
from dynamixel_interfaces.srv import GetDataFromDxl

SERVO_NAME = {1: "arm_r_joint1", 31: "arm_l_joint1"}


class MotorProfiler(Node):
    def __init__(self, args):
        super().__init__("motor_profiler")
        self.args = args
        self.out = os.path.join(args.out_root, time.strftime("motor_profile_%Y%m%d_%H%M%S"))
        os.makedirs(self.out, exist_ok=True)

        self.js_file = open(os.path.join(self.out, "joint_states.csv"), "w", newline="")
        self.js_writer = None
        self.names = None
        self.window = collections.deque()  # (t, {joint: |effort|}) for the 30 s summary

        self.dxl_file = open(os.path.join(self.out, "dxl_state.csv"), "w", newline="")
        self.dxl_writer = csv.writer(self.dxl_file)
        self.dxl_writer.writerow(["t_unix", "comm_state", "id", "torque_on", "hw_error"])
        self.last_dxl = None
        self.last_dxl_write = 0.0
        self.hw_errors = {}

        self.temp_file = open(os.path.join(self.out, "temps.csv"), "w", newline="")
        self.temp_writer = csv.writer(self.temp_file)
        self.temp_writer.writerow(["t_unix", "id", "item", "value", "ok"])
        self.latest = {}

        self.create_subscription(JointState, "/joint_states", self.on_js, qos_profile_sensor_data)
        self.create_subscription(DynamixelState, "/ffw_follower/dxl_state", self.on_dxl, 10)
        self.cli = self.create_client(GetDataFromDxl, "/ffw_follower/get_dxl_data")
        self.pending = 0

        self.create_timer(30.0, self.summary)
        self.get_logger().warning(f"Recording to {self.out}  (Ctrl+C to stop)")
        if args.poll > 0:
            # WARNING: on this driver each get_dxl_data call blocks the follower's 100 Hz
            # read/write loop until it answers (~1 s timeout observed 2026-11-26). Off by default.
            self.create_timer(args.poll, self.poll)
            self.poll()

    def on_js(self, msg):
        t = time.time()
        if self.js_writer is None:
            self.names = list(msg.name)
            self.js_writer = csv.writer(self.js_file)
            self.js_writer.writerow(
                ["t_unix", "t_msg"]
                + [n + "_pos" for n in self.names]
                + [n + "_vel" for n in self.names]
                + [n + "_eff" for n in self.names])
        if list(msg.name) != self.names:
            return
        n = len(self.names)
        vel = list(msg.velocity) if len(msg.velocity) == n else [float("nan")] * n
        eff = list(msg.effort) if len(msg.effort) == n else [float("nan")] * n
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.js_writer.writerow([f"{t:.4f}", f"{stamp:.4f}"]
                                + [f"{v:.5f}" for v in list(msg.position) + vel + eff])
        self.window.append((t, dict(zip(self.names, (abs(e) for e in eff)))))
        while self.window and t - self.window[0][0] > 30.0:
            self.window.popleft()

    def on_dxl(self, msg):
        t = time.time()
        rows = list(zip(msg.id, msg.torque_state, msg.dxl_hw_state))
        key = (msg.comm_state, tuple(rows))
        for i, _tq, hw in rows:
            if hw:
                self.hw_errors[i] = hw
        if key != self.last_dxl or t - self.last_dxl_write >= 1.0:
            for i, tq, hw in rows:
                self.dxl_writer.writerow([f"{t:.3f}", msg.comm_state, i, int(tq), hw])
            if key != self.last_dxl and self.last_dxl is not None:
                self.get_logger().warning(f"dxl_state changed: comm_state={msg.comm_state} "
                                          f"torque_on={[i for i, tq, _ in rows if tq]} "
                                          f"hw_errors={[(i, hw) for i, _, hw in rows if hw]}")
            self.last_dxl, self.last_dxl_write = key, t

    def poll(self):
        if not self.cli.service_is_ready():
            self.get_logger().warning("get_dxl_data service not available yet; skipping this poll")
            return
        if self.pending:
            return  # previous poll still in flight; never stack requests on the driver
        for i in self.args.ids:
            for item in ("Present Motor Temperature", "Present Inverter Temperature", "Error Code"):
                req = GetDataFromDxl.Request()
                req.id, req.item_name, req.timeout_sec = i, item, 1.0
                self.pending += 1
                fut = self.cli.call_async(req)
                fut.add_done_callback(lambda f, i=i, item=item: self.on_reply(f, i, item))

    def on_reply(self, fut, i, item):
        self.pending -= 1
        t = time.time()
        try:
            r = fut.result()
            val, ok = r.item_data, r.result
        except Exception as e:  # noqa: BLE001
            val, ok = str(e), False
        self.temp_writer.writerow([f"{t:.3f}", i, item, val, int(bool(ok))])
        self.temp_file.flush()
        self.latest[(i, item)] = val if ok else "fail"

    def summary(self):
        for f in (self.js_file, self.dxl_file, self.temp_file):
            f.flush()
        if not self.window:
            self.get_logger().warning("no /joint_states received in the last 30 s")
            return
        sums = collections.defaultdict(float)
        for _, d in self.window:
            for k, v in d.items():
                sums[k] += v
        mean = {k: v / len(self.window) for k, v in sums.items()}
        top = sorted(((v, k) for k, v in mean.items() if "wheel" not in k), reverse=True)[:4]
        temps = " ".join(f"T{i}={self.latest.get((i, 'Present Motor Temperature'), '?')}C/{self.latest.get((i, 'Present Inverter Temperature'), '?')}C"
                         for i in self.args.ids)
        err = f" HW_ERRORS={self.hw_errors}" if self.hw_errors else ""
        self.get_logger().warning(
            f"30s mean |current|: R_j1 {mean.get('arm_r_joint1', float('nan')):.1f}  "
            f"L_j1 {mean.get('arm_l_joint1', float('nan')):.1f} | top "
            + ", ".join(f"{k} {v:.1f}" for v, k in top) + f" | {temps}{err}")

    def close(self):
        for f in (self.js_file, self.dxl_file, self.temp_file):
            f.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ids", type=int, nargs="+", default=[1, 31],
                   help="servo IDs to read temperature from (default 1 31 = right/left shoulder)")
    p.add_argument("--poll", type=float, default=0.0,
                   help="seconds between temperature reads; 0 = off (default). Reads stall the driver loop.")
    p.add_argument("--out-root", default="/workspace")
    args = p.parse_args()
    rclpy.init()
    node = MotorProfiler(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print(f"Saved to {node.out}")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
