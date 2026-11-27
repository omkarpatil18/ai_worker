#!/usr/bin/env python3
"""READ-ONLY DYNAMIXEL Protocol 2.0 register reader (standard library only, no pyserial).

Sends only READ instructions (0x02). It never writes a register, never enables or disables torque,
never reboots. Register addresses come from the dynamixel_hardware_interface model files.

Run ONLY with the follower bringup stopped (one program per bus). It refuses to start while a
ros2_control_node is running in this container:
    docker exec -it ai_worker bash -c "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/dxl_readonly_compare.py"
"""
import argparse
import os
import select
import termios
import time

MODEL_DIR = "/root/ros2_ws/src/dynamixel_hardware_interface/param/dxl_model"
ITEMS = [
    "Firmware Version", "Torque Enable", "Present Position", "Homing Offset", "Present Current",
    "Present Input Voltage", "Present Motor Temperature", "Present Inverter Temperature",
    "Present Temperature", "Motor Temperature Limit", "Inverter Temperature Limit", "Temperature Limit",
    "Current Limit", "Position P Gain", "Position I Gain", "Position D Gain", "Velocity P Gain",
    "Velocity I Gain", "Drive Mode", "Safe Stop Time", "Brake Delay", "Profile Time", "Profile Acceleration Time",
    "Error Code", "Error Code History 1", "Error Code History 2", "Error Code History 3",
    "Hardware Error Status",
]


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Bus:
    def __init__(self, dev, baud):
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        speed = getattr(termios, f"B{baud}")
        a[0] = 0; a[1] = 0; a[3] = 0                      # raw iflag/oflag/lflag
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = speed
        a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def read(self, dxl_id, addr, size, timeout=0.05):
        params = [addr & 0xFF, addr >> 8, size & 0xFF, size >> 8]
        length = len(params) + 3
        pkt = [0xFF, 0xFF, 0xFD, 0x00, dxl_id, length & 0xFF, length >> 8, 0x02] + params
        c = crc16(pkt)
        pkt += [c & 0xFF, c >> 8]
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, bytes(pkt))
        buf = b""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            r, _, _ = select.select([self.fd], [], [], max(0.0, end - time.monotonic()))
            if r:
                buf += os.read(self.fd, 4096)
            i = buf.find(b"\xff\xff\xfd\x00")
            # skip our own echo if the adapter echoes; look for a status packet (0x55)
            while i >= 0 and len(buf) >= i + 9 and buf[i + 7] != 0x55:
                i = buf.find(b"\xff\xff\xfd\x00", i + 1)
            if i >= 0 and len(buf) >= i + 7:
                ln = buf[i + 5] | buf[i + 6] << 8
                if len(buf) >= i + 7 + ln:
                    p = buf[i:i + 7 + ln]
                    if p[4] != dxl_id:
                        return None, "wrong id"
                    if crc16(p[:-2]) != (p[-2] | p[-1] << 8):
                        return None, "crc"
                    body = p[8:-2]
                    err = body[0]
                    data = body[1:].replace(b"\xff\xff\xfd\xfd", b"\xff\xff\xfd")  # unstuff
                    return data[:size], err
        return None, "timeout"


def model_files():
    names = {}
    for line in open(os.path.join(MODEL_DIR, "dynamixel.model")):
        parts = line.split()
        if len(parts) == 2 and parts[0].isdigit():
            names[int(parts[0])] = parts[1]
    return names


def control_table(fname):
    table, units = {}, {}
    for line in open(os.path.join(MODEL_DIR, fname)):
        parts = line.rstrip("\n").split("\t")
        if len(parts) >= 3 and parts[0].isdigit() and parts[1].isdigit():
            table[parts[2]] = (int(parts[0]), int(parts[1]))
        elif len(parts) >= 4 and not parts[0].isdigit():
            try:
                units[parts[0]] = (float(parts[1]), parts[2], parts[3] == "signed")
            except ValueError:
                pass
    return table, units


def to_int(data, signed):
    return int.from_bytes(data, "little", signed=signed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/follower")
    ap.add_argument("--baud", type=int, default=4000000)
    ap.add_argument("--pairs", default="1:31,2:32,3:33,4:34,7:37", help="right:left ID pairs")
    ap.add_argument("--force", action="store_true",
                    help="run even though a ros2_control_node is running (NOT recommended)")
    a = ap.parse_args()
    # Only one program may use the servo bus. Talking on it while the driver runs corrupts the
    # driver's 100 Hz read/write; after 500 ms of read errors it drops torque and the arm falls.
    running = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/comm") as f:
                if f.read().strip().startswith("ros2_control_no"):
                    running.append(pid)
        except OSError:
            pass
    if running and not a.force:
        raise SystemExit(
            f"refusing: ros2_control_node is running (pid {', '.join(running)}). Stop the bringup "
            "first; this tool must have the servo bus to itself.")
    pairs = [tuple(int(x) for x in p.split(":")) for p in a.pairs.split(",")]
    bus = Bus(a.dev, a.baud)
    names = model_files()
    result = {}
    for dxl_id in sorted({i for p in pairs for i in p}):
        d, err = bus.read(dxl_id, 0, 2)
        if d is None:
            result[dxl_id] = {"error": f"no reply to model-number read ({err})"}
            continue
        model = to_int(d, False)
        fname = names.get(model, "?")
        r = {"Model Number": model, "Model file": fname, "status-packet error byte": err}
        if fname != "?":
            table, units = control_table(fname)
            for item in ITEMS:
                if item not in table:
                    continue
                addr, size = table[item]
                d, e = bus.read(dxl_id, addr, size)
                if d is None:
                    r[item] = f"read failed ({e})"
                    continue
                signed = units.get(item, (1, "", item in ("Present Current", "Present Position", "Homing Offset")))[2]
                raw = to_int(d, signed)
                if item in units and units[item][1] not in ("raw", ""):
                    r[item] = f"{raw} ({raw * units[item][0]:.3f} {units[item][1]})"
                else:
                    r[item] = raw
        result[dxl_id] = r
        time.sleep(0.01)
    os.close(bus.fd)

    for rid, lid in pairs:
        R, L = result.get(rid, {}), result.get(lid, {})
        print(f"\n=== right ID {rid}  vs  left ID {lid} ===")
        for k in list(dict.fromkeys(list(R) + list(L))):
            rv, lv = R.get(k, "-"), L.get(k, "-")
            flag = "" if str(rv) == str(lv) or k in ("Present Position", "Present Current") else "   <-- differs"
            print(f"  {k:30s} R: {str(rv):28s} L: {str(lv):28s}{flag}")


if __name__ == "__main__":
    main()
