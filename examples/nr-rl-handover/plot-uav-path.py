#!/usr/bin/env python3
"""Plot the UAV path in 3D and top-down view.
   Intermediate points (periodic) = light gray.
   Waypoints (CourseChange) = colored by altitude with labels.
   gNB antennas (from gnb-antennas.csv) = markers + translucent sector cones
   oriented along each antenna's bearing angle."""

import os
import math

import pandas as pd

import matplotlib.pyplot as plt
import matplotlib.patheffects as path_effects
from matplotlib.patches import Wedge
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

script_dir = os.path.dirname(os.path.abspath(__file__))

MOBILITY_FILE = script_dir + '/output/mobility.csv'
ANTENNA_FILE = script_dir + '/output/gnb-antennas.csv'
META_FILE = script_dir + '/output/meta.yaml'

if not os.path.exists(MOBILITY_FILE):
    print(f"MOBILITY data file not found: {MOBILITY_FILE}")
    print("Run 'ns3 run defiance-nr-rl-handover' first to generate it.")
    exit(1)

# ---- gNB antenna positions/orientations (optional overlay) ----
antennas = []
if os.path.exists(ANTENNA_FILE):
    with open(ANTENNA_FILE) as f:
        header = f.readline().strip().split(',')
        for line in f:
            vals = line.strip().split(',')
            if len(vals) < 6:
                continue
            antennas.append({
                'cellId': int(vals[0]),
                'x': float(vals[1]),
                'y': float(vals[2]),
                'z': float(vals[3]),
                'bearing': float(vals[4]),  # degrees, 0 = +x, CCW
                'downtilt': float(vals[5]),
            })
    print(f"Loaded {len(antennas)} gNB antennas from {ANTENNA_FILE}")
else:
    print(f"NOTE: {ANTENNA_FILE} not found — skipping antenna overlay "
          f"(enable --logging in the simulation).")

# Cone radius scales with the inter-site distance from meta.yaml (fallback 125 m)
intersite_distance = 500.0
try:
    with open(META_FILE) as f:
        for line in f:
            if line.startswith('intersiteDistance:'):
                intersite_distance = float(line.split(':', 1)[1].strip())
except OSError:
    pass
CONE_RADIUS = 0.25 * intersite_distance
HALF_BEAM_DEG = 35.0  # half aperture of the drawn cone (sector ~ 65 deg HPBW)

mobility_data = pd.read_csv(
    MOBILITY_FILE, header=None, names=["time", "position", "isWaypoint"])

# Split into waypoints (CourseChange events) and intermediate (periodic) points
waypointData = mobility_data[mobility_data["isWaypoint"] == 1]
pathData = mobility_data[mobility_data["isWaypoint"] == 0]

# Parse positions: x:y:z
all_xs, all_ys, all_zs = zip(*[map(float, line.split(":"))
                               for line in mobility_data["position"]])
wp_xs, wp_ys, wp_zs = [], [], []
if len(waypointData) > 0:
    wp_xs, wp_ys, wp_zs = zip(*[map(float, line.split(":"))
                                for line in waypointData["position"]])

# Filter out waypoints that coincide with start or end position (within 1m)
start_pos = (all_xs[0], all_ys[0], all_zs[0])
end_pos = (all_xs[-1], all_ys[-1], all_zs[-1])
filtered_wp = [(x, y, z) for x, y, z in zip(wp_xs, wp_ys, wp_zs)
               if (abs(x - start_pos[0]) > 1 or abs(y - start_pos[1]) > 1 or abs(z - start_pos[2]) > 1)
               and (abs(x - end_pos[0]) > 1 or abs(y - end_pos[1]) > 1 or abs(z - end_pos[2]) > 1)]
if filtered_wp:
    wp_xs, wp_ys, wp_zs = zip(*filtered_wp)

# Site colors for the sector cones: gNBs are installed site-major, 3 sectors
# each, so cell index // 3 groups sectors of the same site.
SITE_COLORS = plt.cm.Set2.colors

def site_color(gnb_idx):
    return SITE_COLORS[(gnb_idx // 3) % len(SITE_COLORS)]

def draw_antenna_cones(ax):
    """Overlay translucent sector cones oriented along each antenna's bearing."""
    for idx, gnb in enumerate(antennas):
        t1 = gnb['bearing'] - HALF_BEAM_DEG
        t2 = gnb['bearing'] + HALF_BEAM_DEG
        wedge = Wedge((gnb['x'], gnb['y']), CONE_RADIUS, t1, t2,
                      facecolor=site_color(idx), alpha=0.15, edgecolor='none', zorder=2)
        ax.add_patch(wedge)
        ax.scatter([gnb['x']], [gnb['y']], color='black', s=25,
                   marker='^', zorder=4)
        # Label inside the cone, offset along the bearing so the 3 sectors
        # of a co-located site do not overlap (site antennas are ~1 m apart)
        label_frac = 0.55 * CONE_RADIUS
        lx = gnb['x'] + label_frac * math.cos(math.radians(gnb['bearing']))
        ly = gnb['y'] + label_frac * math.sin(math.radians(gnb['bearing']))
        txt = ax.text(lx, ly, f"{gnb['cellId']}",
                      fontsize=7, color='black', ha='center', va='center', zorder=5)
        txt.set_path_effects([path_effects.withStroke(linewidth=2, foreground='white')])

fig = plt.figure(figsize=(14, 6))

# ---- 3D view ----
ax = fig.add_subplot(121, projection="3d")

# Intermediate path points: tiny, light gray (lowest zorder)
ax.scatter(all_xs, all_ys, all_zs, color="lightgray", s=1, label="Intermediate", zorder=1)
# Path line: light gray, thin
ax.plot(all_xs, all_ys, all_zs, color="lightgray", alpha=0.5, linewidth=0.8, zorder=1)

# gNB antennas (position only, for 3D context)
if antennas:
    ax.scatter([g['x'] for g in antennas], [g['y'] for g in antennas],
               [g['z'] for g in antennas], color='black', s=40,
               marker='^', label='gNB antenna', zorder=2)

# Waypoints: colored by altitude, larger, with altitude labels (higher zorder)
if len(wp_xs) > 0:
    wp_sc = ax.scatter(wp_xs, wp_ys, wp_zs, c=wp_zs, cmap="plasma",
                       s=60, edgecolors="black", linewidth=0.5, label="Waypoint", zorder=3)
    for wx, wy, wz in zip(wp_xs, wp_ys, wp_zs):
        txt = ax.text(wx, wy, wz + 10, f"z={wz:.1f}",
                      fontsize=8, color="black", ha="center", zorder=4)
        txt.set_path_effects([path_effects.withStroke(linewidth=3, foreground="white")])
    fig.colorbar(wp_sc, ax=ax, label="Altitude (m)", shrink=0.6)

# Start / End markers (topmost zorder) with altitude labels
sx, sy, sz = all_xs[0], all_ys[0], all_zs[0]
ex, ey, ez = all_xs[-1], all_ys[-1], all_zs[-1]
ax.scatter([sx], [sy], [sz], color="green",
           s=80, marker="o", label="Start", zorder=5)
ax.scatter([ex], [ey], [ez], color="red",
           s=80, marker="^", label="End", zorder=5)
txt_s = ax.text(sx, sy, sz + 10, f"z={sz:.1f}",
                fontsize=8, color="black", ha="center", zorder=4)
txt_s.set_path_effects([path_effects.withStroke(linewidth=3, foreground="white")])
txt_e = ax.text(ex, ey, ez + 10, f"z={ez:.1f}",
                fontsize=8, color="black", ha="center", zorder=4)
txt_e.set_path_effects([path_effects.withStroke(linewidth=3, foreground="white")])

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("3D View")
ax.set_zlim(0, 300)
ax.legend(loc="upper left")

# ---- Top-down view ----
ax2 = fig.add_subplot(122)

# Intermediate path points: light gray (lowest zorder)
ax2.scatter(all_xs, all_ys, color="lightgray", s=1, label="Intermediate", zorder=1)
ax2.plot(all_xs, all_ys, color="lightgray", alpha=0.5, linewidth=0.8, zorder=1)

# gNB antennas: translucent sector cones + markers (over the path, under waypoints)
if antennas:
    draw_antenna_cones(ax2)

# Waypoints: colored by altitude, larger (higher zorder)
if len(wp_xs) > 0:
    wp_sc2 = ax2.scatter(wp_xs, wp_ys, c=wp_zs, cmap="plasma",
                         s=60, edgecolors="black", linewidth=0.5, label="Waypoint", zorder=3)
    fig.colorbar(wp_sc2, ax=ax2, label="Altitude (m)", shrink=0.6)

# Start / End markers (topmost zorder) with altitude labels
ax2.scatter([sx], [sy], color="green", s=80, marker="o", label="Start", zorder=5)
ax2.text(sx, sy - 25, f"z={sz:.1f}", fontsize=8, color="green", ha="center", zorder=6)
ax2.scatter([ex], [ey], color="red", s=80, marker="^", label="End", zorder=5)
ax2.text(ex, ey - 25, f"z={ez:.1f}", fontsize=8, color="red", ha="center", zorder=6)

ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.set_title("Top-Down View (XY) — sector cones = gNB bearing")
ax2.set_aspect("equal")
if antennas:
    from matplotlib.lines import Line2D
    ax2.legend(handles=[Line2D([0], [0], marker='^', color='none', markerfacecolor='black',
                                markersize=8, label='gNB antenna'),
                        Wedge((0, 0), 1, 0, 60, facecolor='gray', alpha=0.3,
                              edgecolor='none', label='sector cone')],
               loc="upper left")

plt.tight_layout()
plt.savefig(script_dir + "/output/uav_path.png", dpi=150)
print("Saved uav_path.png")
plt.show()
