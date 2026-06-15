#!/usr/bin/env python3
"""
Validation of the ns-3 ThreeGppAntennaModelOriented (3GPP TR 38.901 Outdoor).

Shows how the radiation pattern shifts with orientation (azimuth) and downtilt.

The oriented model applies:
    phi_eff   = phi - orientation   (wrapped to [-180, 180])
    theta_eff = theta - downtilt

where:
  orientation  = antenna horizontal pointing direction (degrees)
  downtilt     = downward tilt angle (degrees, positive = below horizon)

Outdoor pattern (ITU-R M.2412):
  phi_3dB=65, theta_3dB=65, A_max=30, SLA_V=30, G_E,max=8

Usage:
    python3 plot-three-gpp-antenna.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.gridspec import GridSpec

# ─── Outdoor pattern parameters (3GPP TR 38.901 Table 7.3-1) ─────────
OUTDOOR = {
    "phi_3dB": 65.0,
    "theta_3dB": 65.0,
    "A_max": 30.0,
    "SLA_V": 30.0,
    "G_E_max": 8.0,
}


def three_gpp_gain_db(phi_deg, theta_deg, params=None):
    """
    Base 3GPP antenna gain (dB). No orientation/downtilt.
    """
    if params is None:
        params = OUTDOOR
    phi = np.asarray(phi_deg, dtype=float)
    theta = np.asarray(theta_deg, dtype=float)

    vert_gain = -np.minimum(params["SLA_V"],
                            12.0 * ((theta - 90.0) / params["theta_3dB"]) ** 2)
    horiz_gain = -np.minimum(params["A_max"],
                             12.0 * (phi / params["phi_3dB"]) ** 2)

    gain = params["G_E_max"] - np.minimum(params["A_max"],
                                          -(vert_gain + horiz_gain))
    return gain


def three_gpp_gain_db_oriented(phi_deg, theta_deg,
                                orientation_deg=0.0, downtilt_deg=0.0,
                                params=None):
    """
    Oriented 3GPP antenna gain (dB).

    Mirrors ThreeGppAntennaModelOriented::GetGainDb() in C++:

        phi_eff   = wrap180(phi - orientation)
        theta_eff = theta - downtilt
        gain      = three_gpp_gain_db(phi_eff, theta_eff, params)
    """
    if params is None:
        params = OUTDOOR
    phi = np.asarray(phi_deg, dtype=float)
    theta = np.asarray(theta_deg, dtype=float)

    phi_eff = (phi - orientation_deg + 180.0) % 360.0 - 180.0
    theta_eff = theta - downtilt_deg

    return three_gpp_gain_db(phi_eff, theta_eff, params)


def plot_3d_pattern(ax, orientation_deg=0.0, downtilt_deg=0.0,
                    color_map=cm.viridis):
    """
    Plot a 3D surface of the oriented antenna pattern on *ax*.
    """
    theta = np.linspace(0, np.pi, 70)
    phi = np.linspace(0, 2 * np.pi, 100)
    Theta, Phi_mesh = np.meshgrid(theta, phi)

    theta_deg = np.rad2deg(Theta)
    phi_deg = np.rad2deg(Phi_mesh)
    phi_deg = (phi_deg + 180) % 360 - 180

    gain_db = three_gpp_gain_db_oriented(phi_deg, theta_deg,
                                          orientation_deg, downtilt_deg)
    gain_lin = 10.0 ** (gain_db / 20.0)

    X = gain_lin * np.sin(Theta) * np.cos(Phi_mesh)
    Y = gain_lin * np.sin(Theta) * np.sin(Phi_mesh)
    Z = gain_lin * np.cos(Theta)

    norm = plt.Normalize(gain_db.min(), gain_db.max())
    ax.plot_surface(X, Y, Z,
                    facecolors=color_map(norm(gain_db)),
                    rstride=1, cstride=1,
                    alpha=0.85, linewidth=0, antialiased=True)

    # Reference sphere
    u = np.linspace(0, 2 * np.pi, 25)
    v = np.linspace(0, np.pi, 25)
    U, V = np.meshgrid(u, v)
    Xs = 0.2 * np.sin(V) * np.cos(U)
    Ys = 0.2 * np.sin(V) * np.sin(U)
    Zs = 0.2 * np.cos(V)
    ax.plot_wireframe(Xs, Ys, Zs, color="gray", alpha=0.1, linewidth=0.5)

    max_r = gain_lin.max() * 1.15
    ax.set_xlim(-max_r, max_r)
    ax.set_ylim(-max_r, max_r)
    ax.set_zlim(-max_r, max_r)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Ori={orientation_deg}°  Tilt={downtilt_deg}°",
                 fontsize=10)

    return norm


# ─── Build figure ────────────────────────────────────────────────────────
fig = plt.figure(figsize=(20, 14), constrained_layout=True)
gs = GridSpec(2, 4, figure=fig,
              width_ratios=[1, 1, 1, 1],
              height_ratios=[1.2, 1],
              hspace=0.30, wspace=0.25)

# ═══════════════════════════════════════════════════════════════════════
# TOP ROW: 3D surfaces — orientation sweep at downtilt = 0
# ═══════════════════════════════════════════════════════════════════════
axes_top = [fig.add_subplot(gs[0, i], projection="3d") for i in range(4)]
orientations = [0, 45, 90, 120]

for ax, o in zip(axes_top, orientations):
    plot_3d_pattern(ax, orientation_deg=o, downtilt_deg=0)

fig.suptitle("ThreeGppAntennaModelOriented — Outdoor Pattern Validation",
             fontsize=14, y=1.01)

# ═══════════════════════════════════════════════════════════════════════
# BOTTOM ROW
#   col 0-1: 3D surfaces — same orientations at downtilt = 10
#   col 2:   azimuth cuts (tilt=0 solid, tilt=10 dashed)
#   col 3:   vertical cuts showing tilt shift
# ═══════════════════════════════════════════════════════════════════════
ax_bot_0 = fig.add_subplot(gs[1, 0], projection="3d")
ax_bot_1 = fig.add_subplot(gs[1, 1], projection="3d")
ax_az_cut = fig.add_subplot(gs[1, 2])
ax_vt_cut = fig.add_subplot(gs[1, 3])

# --- Bottom-left 3D: Ori=0°, Tilt=10° and Ori=45°, Tilt=10° ---
plot_3d_pattern(ax_bot_0, orientation_deg=0, downtilt_deg=10)
plot_3d_pattern(ax_bot_1, orientation_deg=45, downtilt_deg=10)

# --- Azimuth cut: all orientations at tilt=0 (solid) and tilt=10 (dashed) ---
phi_1d = np.linspace(-180, 180, 721)
styles = [(0, "-"), (10, "--")]
colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

for i, o in enumerate([0, 45, 90, 120]):
    for dt, ls in styles:
        gain_db = three_gpp_gain_db_oriented(phi_1d, 90.0,
                                              orientation_deg=o,
                                              downtilt_deg=dt)
        label = f"Ori={o}°" if dt == 0 else None
        ax_az_cut.plot(phi_1d, gain_db, linestyle=ls, color=colors[i],
                       linewidth=1.5 if dt == 0 else 1.2,
                       alpha=0.9 if dt == 0 else 0.6,
                       label=label)

handles, labels = ax_az_cut.get_legend_handles_labels()
handles += [plt.Line2D([], [], color="gray", linestyle="-", linewidth=1.5),
            plt.Line2D([], [], color="gray", linestyle="--", linewidth=1.2)]
labels += ["Tilt=0°", "Tilt=10°"]

ax_az_cut.legend(handles, labels, fontsize=7)
ax_az_cut.set_xlabel("Azimuth (deg)")
ax_az_cut.set_ylabel("Gain (dB)")
ax_az_cut.set_title("Azimuth Cut at Horizon ($\\theta=90°$)", fontsize=11)
ax_az_cut.grid(True, alpha=0.3)
ax_az_cut.set_xlim(-180, 180)
ax_az_cut.set_ylim(-35, 10)

# Mark orientation peaks
for i, o in enumerate([0, 45, 90, 120]):
    ax_az_cut.axvline(o, color=colors[i], linestyle=":", alpha=0.4)

# --- Vertical cut: tilt variations at boresight ---
theta_1d = np.linspace(0, 180, 721)
tilt_values = [0, 10]
vcolors = ["tab:blue", "tab:orange"]

for dt, c in zip(tilt_values, vcolors):
    gain_db = three_gpp_gain_db_oriented(0.0, theta_1d,
                                          orientation_deg=0,
                                          downtilt_deg=dt)
    ls = "-" if dt == 0 else "--"
    ax_vt_cut.plot(theta_1d, gain_db, linestyle=ls, color=c,
                   linewidth=1.8,
                   label=f"Tilt={dt}°")
    # Mark peak at theta = 90 + dt
    peak_at = 90.0 + dt
    ax_vt_cut.axvline(peak_at, color=c, linestyle=":", alpha=0.5)
    ax_vt_cut.annotate(f"  {peak_at:.0f}°", xy=(peak_at, 7),
                       color=c, fontsize=8, rotation=90, va="bottom")

ax_vt_cut.set_xlabel("Inclination (deg)")
ax_vt_cut.set_ylabel("Gain (dB)")
ax_vt_cut.set_title("Vertical Cut at Boresight ($\\phi=0°$)", fontsize=11)
ax_vt_cut.legend(fontsize=8)
ax_vt_cut.grid(True, alpha=0.3)
ax_vt_cut.set_xlim(0, 180)
ax_vt_cut.set_ylim(-35, 10)

# ─── Validation summary ─────────────────────────────────────────────
valid_text = (
    "Validation checks:\n"
    "1. Azimuth gain peak at phi = orientation (solid lines)\n"
    "2. Azimuth peak position unchanged by downtilt (dashed = tilt=10)\n"
    "3. Vertical gain peak at theta = 90 + downtilt (dotted lines)"
)
ax_sum = fig.add_subplot(gs[1, :], frameon=False)
ax_sum.text(0.5, -0.18, valid_text,
            transform=ax_sum.transAxes,
            fontsize=9, ha="center", va="top",
            bbox=dict(boxstyle="round,pad=0.5",
                      facecolor="lightyellow", alpha=0.8))
ax_sum.set_visible(False)

plt.savefig("three-gpp-antenna-pattern.png", dpi=200, bbox_inches="tight")
print("Saved three-gpp-antenna-pattern.png")

try:
    plt.show()
except Exception:
    pass
