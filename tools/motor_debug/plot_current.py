#!/usr/bin/env python3
"""Plot shoulder current (left vs right) from a motor_profile recording.

    docker exec ai_worker python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/plot_current.py [/workspace/motor_profile_<stamp>] [--last SECONDS]

Writes <dir>/current_shoulders.png. Read-only on the recording; does not touch ROS or the robot.
"""
import csv
import glob
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

args = [a for a in sys.argv[1:] if not a.startswith("--")]
last = float(sys.argv[sys.argv.index("--last") + 1]) if "--last" in sys.argv else None
d = args[0] if args else sorted(glob.glob("/workspace/motor_profile_2*"))[-1]

rows = list(csv.reader(open(d + "/joint_states.csv")))
hdr = rows[0]
data = np.array([[float(x) for x in r] for r in rows[1:] if len(r) == len(hdr)])
col = {h: i for i, h in enumerate(hdr)}
t = data[:, col["t_unix"]] - data[0, col["t_unix"]]
if last:
    keep = t >= t[-1] - last
    data, t = data[keep], t[keep]


def smooth(x, n=25):  # 0.25 s moving average
    y = np.convolve(x, np.ones(n) / n, mode="same")
    y[:n] = x[:n]; y[-n:] = x[-n:]
    return y


INK, INK2, GRID, SURFACE = "#191919", "#666666", "#e6e6e6", "#ffffff"
SERIES = {"Left shoulder (ID 31)": ("arm_l_joint1", "#257de0"), "Right shoulder (ID 1)": ("arm_r_joint1", "#f47230")}

fig, (ax, ax2) = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True, gridspec_kw={"height_ratios": [3, 1]},
                              facecolor=SURFACE)
for name, (joint, color) in SERIES.items():
    cur = np.abs(smooth(data[:, col[joint + "_eff"]]))
    ax.plot(t, cur, color=color, lw=2, label=name, solid_capstyle="round")
    ax.annotate(f"{name.split(' (')[0]}  {cur[-50:].mean():.0f}", (t[-1], cur[-50:].mean()), xytext=(6, 0),
                textcoords="offset points", va="center", fontsize=9, color=INK)
    ax2.plot(t, data[:, col[joint + "_pos"]] + 1.5994, color=color, lw=2)

ax.set_ylabel("|current| (driver units, 0.25 s average)", color=INK2, fontsize=9)
ax.set_title("Shoulder current, left vs right", loc="left", color=INK, fontsize=13, fontweight="bold", pad=22)
ax.text(0, 1.03, d.split("/")[-1] + ("" if not last else f"  (last {last:.0f} s)"), transform=ax.transAxes,
        color=INK2, fontsize=9)
ax.legend(loc="lower left", frameon=False, fontsize=9, ncol=2, labelcolor=INK)
ax.set_ylim(0, None)
lo, hi = ax2.get_ylim()
ax2.set_ylim(min(lo, -0.1), max(hi, 0.1))
ax2.ticklabel_format(axis="y", style="plain", useOffset=False)
ax2.set_ylabel("shoulder angle\nfrom home (rad)", color=INK2, fontsize=9)
ax2.set_xlabel("time since recording start (s)", color=INK2, fontsize=9)
for a in (ax, ax2):
    a.set_facecolor(SURFACE)
    a.grid(axis="y", color=GRID, lw=0.8)
    a.set_axisbelow(True)
    for s in ("top", "right", "left"):
        a.spines[s].set_visible(False)
    a.spines["bottom"].set_color(GRID)
    a.tick_params(colors=INK2, labelsize=8, length=0)
    a.margins(x=0)
fig.subplots_adjust(right=0.86, hspace=0.12)
out = d + "/current_shoulders.png"
fig.savefig(out, dpi=130, facecolor=SURFACE)
print(out, f"{len(t)} samples, {t[-1]-t[0]:.0f} s")
