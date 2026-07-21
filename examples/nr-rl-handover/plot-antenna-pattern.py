#!/usr/bin/env python3
"""
3D visualization of the ns-3 ParabolicAntennaModel.

The ParabolicAntennaModel gain (in dB) is:

    G(phi) = -min( 12 * (phi / theta_3dB)^2 ,  A_max )

where:
  phi       = azimuth angle relative to boresight (radians)
  theta_3dB = 3 dB beamwidth (radians)
  A_max     = maximum attenuation (dB)

This model has no elevation dependence — the gain is constant
for all inclination angles at a given azimuth offset.

Usage:
    python3 plot-antenna-pattern.py

Adjust BEAMWIDTH, MAX_ATTENUATION, and ORIENTATION below.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm


# ─── User-configurable parameters ───────────────────────────────────────
BEAMWIDTH_DEG_PHI = 60.0        # 3 dB beamwidth (degrees)
BEAMWIDTH_DEG_THETA = 60.0      # 3 dB beamwidth (degrees)
MAX_ATTENUATION_DB = 20.0   # maximum attenuation (dB)
ORIENTATION_DEG = 0.0       # boresight direction (degrees)
# ────────────────────────────────────────────────────────────────────────


def parabolic_gain_db(phi_deg: np.ndarray,
                      theta_deg: np.ndarray,
                      beamwidth_phi_deg: float,
                      beamwidth_theta_deg: float,
                      max_atten: float) -> np.ndarray:
    """
    Compute the ParabolicAntennaModel gain in dB.

    Parameters
    ----------
    phi_deg : ndarray  Azimuth offset from boresight (degrees).
    theta_deg : ndarray  Inclination angle (degrees).
    beamwidth_deg : float  3 dB beamwidth (degrees).
    max_atten : float      Maximum attenuation (dB).

    Returns
    -------
    ndarray  Gain (dB).
    """
    phi_rad = np.deg2rad(phi_deg)
    theta_rad = np.deg2rad(theta_deg - 100.0)
    bw_phi_rad = np.deg2rad(beamwidth_phi_deg)
    bw_theta_rad = np.deg2rad(beamwidth_theta_deg)
    return -np.minimum(12.0 * ((phi_rad / bw_phi_rad) ** 2) + 12.0 * ((theta_rad / bw_theta_rad) ** 2), max_atten)


# ─── Build the 3D radiation pattern ─────────────────────────────────────
# Spherical grid: theta = inclination (0 at +z), phi = azimuth (0 along +x)
theta = np.linspace(0, 2*np.pi, 120)        # inclination
phi = np.linspace(0, 2 * np.pi, 120)     # azimuth
Theta, Phi = np.meshgrid(theta, phi)

# Azimuth offset from boresight (orientation points along the +x axis)
phi_offset_deg = np.rad2deg(Phi) - ORIENTATION_DEG
# Normalise to [-180, 180]
phi_offset_deg = (phi_offset_deg + 180) % 360 - 180

theta_deg = np.rad2deg(Theta)

# Gain (dB)
gain_db = parabolic_gain_db(phi_offset_deg, theta_deg, BEAMWIDTH_DEG_PHI, BEAMWIDTH_DEG_THETA, MAX_ATTENUATION_DB)

# Convert to linear gain for the radius in the spherical plot
gain_linear = 10.0 ** (gain_db / 20.0)

# Cartesian coordinates of the pattern surface
X = gain_linear * np.sin(Theta) * np.cos(Phi)
Y = gain_linear * np.sin(Theta) * np.sin(Phi)
Z = gain_linear * np.cos(Theta)


# ─── Also create a 2D polar cut in the azimuth plane ────────────────────
phi_1d = np.linspace(-180, 180, 721)
gain_db_1d = parabolic_gain_db(phi_1d, 0.0, BEAMWIDTH_DEG_PHI, BEAMWIDTH_DEG_THETA, MAX_ATTENUATION_DB)
gain_lin_1d = 10.0 ** (gain_db_1d / 20.0)
phi_1d_rad = np.deg2rad(phi_1d)

# ─── Plot ───────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 7))

# -- 3D surface plot (left) --
ax1 = fig.add_subplot(1, 2, 1, projection="3d")
# Colour by dB value
norm = plt.Normalize(gain_db.min(), gain_db.max())
surf = ax1.plot_surface(X, Y, Z,
                        facecolors=cm.viridis(norm(gain_db)),
                        rstride=1, cstride=1,
                        alpha=0.9, linewidth=0, antialiased=True)

# Add a faint unit sphere for reference
u = np.linspace(0, 2 * np.pi, 40)
v = np.linspace(0, np.pi, 40)
U, V = np.meshgrid(u, v)
Xs = 0.3 * np.sin(V) * np.cos(U)
Ys = 0.3 * np.sin(V) * np.sin(U)
Zs = 0.3 * np.cos(V)
ax1.plot_wireframe(Xs, Ys, Zs, color="gray", alpha=0.15, linewidth=0.5)

# Axes
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_zlabel("Z")
max_r = gain_linear.max() * 1.15
ax1.set_xlim(-max_r, max_r)
ax1.set_ylim(-max_r, max_r)
ax1.set_zlim(-max_r, max_r)
ax1.set_title(f"3D Radiation Pattern\n"
              f"(Beamwidth = {BEAMWIDTH_DEG_PHI}°, Max Attn = {MAX_ATTENUATION_DB} dB, "
              f"Orientation = {ORIENTATION_DEG}°)",
              fontsize=11)

# Colour bar
mappable = cm.ScalarMappable(norm=norm, cmap=cm.viridis)
mappable.set_array(gain_db)
cbar = fig.colorbar(mappable, ax=ax1, shrink=0.6, pad=0.1)
cbar.set_label("Gain (dB)")

# -- 2D polar plot (right) --
ax2 = fig.add_subplot(1, 2, 2, projection="polar")
ax2.plot(phi_1d_rad, gain_lin_1d, color="tab:blue", linewidth=2)
ax2.fill(phi_1d_rad, gain_lin_1d, alpha=0.25, color="tab:blue")
ax2.set_title(f"Azimuth Cut\n(Beamwidth = {BEAMWIDTH_DEG_PHI}°, Orientation = {ORIENTATION_DEG}°)",
              fontsize=11, pad=20)
ax2.set_ylim(0, gain_lin_1d.max() * 1.15)
ax2.set_yticks([])  # hide radial tick labels for clarity

# Annotate beamwidth on the polar plot
half_power = 10.0 ** (-3.0 / 20.0)  # linear gain at -3 dB
ax2.plot([-np.deg2rad(BEAMWIDTH_DEG_PHI / 2), np.deg2rad(BEAMWIDTH_DEG_PHI / 2)],
         [half_power, half_power],
         color="red", linestyle="--", linewidth=1.5, label=f"3 dB beamwidth ({BEAMWIDTH_DEG_PHI}°)")
ax2.legend(loc="upper right", fontsize=9)

plt.tight_layout()
plt.savefig("antenna-pattern-parabolic.png", dpi=200, bbox_inches="tight")
print("Saved antenna-pattern-parabolic.png")

# Try to show interactively; non-interactive backends (e.g. headless) will
# print a warning instead of crashing.
try:
    plt.show()
except Exception:
    pass
