#!/usr/bin/env python3
"""Overload watchdog for the FFW-SG2 follower.

When any DYNAMIXEL on the follower reports an error, switch torque off on ALL follower servos at
once (the same thing that happens when the bringup is killed), instead of letting one joint go
limp while the others stay stiff and keep following commands.

Detection: the driver (dynamixel_hardware_interface) logs every servo fault on /rosout, e.g.
    "Dynamixel Error Code [ ID:1] --> 0x5 (Overload Error): ..."
    "Dynamixel Hardware Error States [ ID:7] --> 0x..."
(/ffw_follower/dxl_state can't be used: it showed dxl_hw_state 0 while ID 1 was latched.)

Action: calls the driver's own service  dynamixel_hardware_interface/set_dxl_torque  (std_srvs/SetBool,
data=false) = "Disabling torque for all Dynamixels". Only once. To recover, restart the bringup.

Modes:
    --dry-run   (DEFAULT) only prints what it would do; never calls the service
    --arm       actually calls the torque-off service on the first fault

Run inside the ai_worker container while the bringup is up:
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/overload_watchdog.py"          # dry run
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/overload_watchdog.py --arm"    # live
"""
import argparse
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Log
from std_srvs.srv import SetBool

PATTERN = re.compile(r"Dynamixel (Error Code|Hardware Error States) \[ ID:(\d+)\] --> (0x[0-9a-fA-F]+)")
DRIVER_LOGGER = "dynamixel_hardware_interface"
TORQUE_SRV = "/dynamixel_hardware_interface/set_dxl_torque"


class Watchdog(Node):
    def __init__(self, armed, test_text):
        super().__init__("overload_watchdog")
        self.armed = armed
        self.tripped = False
        self.seen_driver = False
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.start = time.time()
        self.create_subscription(Log, "/rosout", self.on_log, qos)
        self.cli = self.create_client(SetBool, TORQUE_SRV)
        self.create_timer(10.0, self.heartbeat)
        mode = "ARMED: will switch ALL follower torque off on the first servo fault" if armed else \
            "DRY RUN: will only report what it would do"
        self.get_logger().warning(f"{mode}. Watching /rosout for {DRIVER_LOGGER} fault messages.")
        if test_text:  # self-test of the matcher, never acts
            m = PATTERN.search(test_text)
            self.get_logger().warning(f"self-test: pattern {'MATCHES' if m else 'does NOT match'}: {test_text!r}")

    def heartbeat(self):
        svc = "available" if self.cli.service_is_ready() else "NOT available"
        drv = "seen" if self.seen_driver else "NOT seen yet (driver only logs on events; that is normal)"
        self.get_logger().info(f"alive; torque service {svc}; driver log messages {drv}")

    def on_log(self, msg):
        if DRIVER_LOGGER not in msg.name:
            return
        self.seen_driver = True
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if stamp < self.start - 1.0:
            return  # old message replayed by transient-local durability; ignore
        m = PATTERN.search(msg.msg)
        if not m or self.tripped:
            return
        self.tripped = True
        kind, sid, code = m.groups()
        self.get_logger().error(f"FAULT on servo ID {sid}: {kind} {code}  ({msg.msg[:120]})")
        if not self.armed:
            self.get_logger().warning(f"DRY RUN: would now call {TORQUE_SRV} data=false (all torque off)")
            return
        if not self.cli.service_is_ready():
            self.get_logger().error(f"{TORQUE_SRV} not available; could NOT switch torque off")
            return
        req = SetBool.Request()
        req.data = False
        fut = self.cli.call_async(req)
        fut.add_done_callback(self.on_done)
        self.get_logger().warning("torque-off requested for all follower servos")

    def on_done(self, fut):
        try:
            r = fut.result()
            self.get_logger().warning(f"torque-off reply: success={r.success} message={r.message!r}. "
                                      "Restart the bringup to recover.")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"torque-off call failed: {e}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", action="store_true", help="actually switch torque off (default: dry run)")
    ap.add_argument("--self-test", action="store_true", help="check the matcher on a sample line at start")
    a = ap.parse_args()
    rclpy.init()
    sample = ("Dynamixel Error Code [ ID:1] --> 0x5 (Overload Error): Operating current exceeding "
              "rated current for an extended duration") if a.self_test else None
    n = Watchdog(a.arm, sample)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
