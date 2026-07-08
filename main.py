"""
Minimal 2D Kalman Filter Demo
=============================
Scenario: a robot moves across a 2D plane with approximately constant velocity.
The only available sensor is a noisy GPS-like position measurement. The true
position and velocity are hidden. The goal is to recursively estimate the
robot's position and velocity with a Kalman Filter.

State vector x = [px, py, vx, vy]^T
  px, py: position
  vx, vy: velocity

This is a small foundation for SLAM and sensor fusion:
  1. Predict the next state with a motion model.
  2. Update that prediction with a new measurement.
"""

import os
from pathlib import Path

cache_dir = Path(".cache")
cache_dir.mkdir(exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(cache_dir / "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(cache_dir))

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

np.random.seed(42)

# ---------- 1. Simulate the real world: a robot follows a curved path ----------
dt = 0.1  # Time interval per step, in seconds
n_steps = 150

t = np.arange(n_steps) * dt
true_px = 5 * np.sin(0.3 * t) + 0.5 * t
true_py = 3 * np.cos(0.2 * t) + 0.3 * t
true_positions = np.stack([true_px, true_py], axis=1)

# ---------- 2. Simulate the sensor: GPS reading = true position + noise ----------
gps_noise_std = 0.8  # GPS standard deviation in meters
measurements = true_positions + np.random.normal(0, gps_noise_std, true_positions.shape)

# ---------- 3. Initialize the Kalman Filter ----------
# State transition matrix F: constant velocity motion model
F = np.array([
    [1, 0, dt, 0],
    [0, 1, 0, dt],
    [0, 0, 1,  0],
    [0, 0, 0,  1],
])

# Observation matrix H: only position (px, py) is observed, not velocity
H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
])

# Process noise Q: uncertainty in the motion model
q = 0.005
Q = q * np.eye(4)

# Measurement noise R: uncertainty in the GPS readings
R = (gps_noise_std ** 2) * np.eye(2)

# Initial state estimate and initial uncertainty
x_est = np.array([measurements[0, 0], measurements[0, 1], 0, 0])
P = np.eye(4) * 10  # Large covariance because the initial estimate is uncertain

estimates = []

# ---------- 4. Main loop: predict, then update ----------
for z in measurements:
    # --- Predict ---
    x_pred = F @ x_est
    P_pred = F @ P @ F.T + Q

    # --- Update ---
    y = z - H @ x_pred                      # Innovation: measurement minus prediction
    S = H @ P_pred @ H.T + R                # Innovation covariance
    K = P_pred @ H.T @ np.linalg.inv(S)     # Kalman gain

    x_est = x_pred + K @ y
    P = (np.eye(4) - K @ H) @ P_pred

    estimates.append(x_est.copy())

estimates = np.array(estimates)

# ---------- 5. Visualize the true path, noisy GPS, and Kalman estimate ----------
plt.figure(figsize=(8, 6))
plt.plot(true_positions[:, 0], true_positions[:, 1],
         label="Ground Truth (true path)", color="black", linewidth=2)
plt.scatter(measurements[:, 0], measurements[:, 1],
            label="Noisy GPS measurements", color="red", s=10, alpha=0.4)
plt.plot(estimates[:, 0], estimates[:, 1],
         label="Kalman Filter estimate", color="blue", linewidth=2)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("2D Kalman Filter: recovering the true trajectory from noisy GPS")
plt.legend()
plt.axis("equal")
plt.grid(alpha=0.3)
plt.tight_layout()

output_dir = Path("results")
output_dir.mkdir(exist_ok=True)
output_path = output_dir / "kalman_filter_demo.png"
plt.savefig(output_path, dpi=150)

# ---------- 6. Quantitative evaluation: compare GPS and Kalman accuracy ----------
rmse_measurement = np.sqrt(np.mean(np.sum((measurements - true_positions) ** 2, axis=1)))
rmse_estimate = np.sqrt(np.mean(np.sum((estimates[:, :2] - true_positions) ** 2, axis=1)))

print(f"Raw GPS measurement RMSE : {rmse_measurement:.3f} m")
print(f"Kalman Filter RMSE       : {rmse_estimate:.3f} m")
print(f"Error reduction          : {(1 - rmse_estimate/rmse_measurement) * 100:.1f}%")
print(f"Saved visualization to   : {output_path}")
