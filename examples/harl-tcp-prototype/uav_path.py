#!/usr/bin/env python3
"""Plot the UAV helix path in 3D and top-down view."""

import os
import pandas as pd

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

script_dir = os.path.dirname(os.path.abspath(__file__))

MOBILITY_FILE = script_dir + '/output/mobility.csv'

if not os.path.exists(MOBILITY_FILE):
        print(f"MOBILITY data file not found: {MOBILITY_FILE}")
        print("Run 'ns3 run defiance-tcp-harl' first to generate it.")
        exit(1)

mobility_data = pd.read_csv(MOBILITY_FILE, header=None, names=["time", "position"])

# Parse: x, y, z
xs, ys, zs = zip(*[map(float, line.split(":")) for line in mobility_data["position"]])

fig = plt.figure(figsize=(14, 6))

# ---- 3D view ----
ax = fig.add_subplot(121, projection="3d")
sc = ax.scatter(xs, ys, zs, c=zs, cmap="plasma", s=20, label="UAV path")
ax.plot(xs, ys, zs, color="gray", alpha=0.3, linewidth=0.8)
ax.scatter([xs[0]], [ys[0]], [zs[0]], color="green", s=80, marker="o", label="Start")
ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color="red", s=80, marker="^", label="End")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("3D View")
# ax.set_xlim(-500, 500)
# ax.set_ylim(-500, 500)
ax.set_zlim(0, 300)
fig.colorbar(sc, ax=ax, label="Altitude (m)")
ax.legend(loc="upper left")

# ---- Top-down view ----
ax2 = fig.add_subplot(122)
sc2 = ax2.scatter(xs, ys, c=zs, cmap="plasma", s=20, label="UAV path")
ax2.plot(xs, ys, color="gray", alpha=0.3, linewidth=0.8)
ax2.scatter([xs[0]], [ys[0]], color="green", s=80, marker="o", label="Start")
ax2.scatter([xs[-1]], [ys[-1]], color="red", s=80, marker="^", label="End")
ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.set_title("Top-Down View (XY)")
# ax2.set_xlim(-500, 500)
# ax2.set_ylim(-500, 500)
ax2.set_aspect("equal")
fig.colorbar(sc2, ax=ax2, label="Altitude (m)")
ax2.legend(loc="upper left")

plt.tight_layout()
plt.savefig(script_dir+ "/output/uav_path.png", dpi=150)
print("Saved uav_path.png")
plt.show()
