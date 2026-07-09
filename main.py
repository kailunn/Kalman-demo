"""
Interactive 2D Kalman Filter Demo (local matplotlib window)
==============================================================
Same logic as kalman_demo.py, except the "GPS noise R" and
"process noise Q" are now draggable sliders. Moving a slider
recomputes the Kalman Filter and redraws the plot live.

Run it on your MacBook with:
    python3 kalman_demo_interactive.py
This opens a real matplotlib window (not a web page) with two
sliders and a button underneath the chart.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

dt = 0.1
n_steps = 150

t = np.arange(n_steps) * dt
true_px = 5 * np.sin(0.3 * t) + 0.5 * t
true_py = 3 * np.cos(0.2 * t) + 0.3 * t
true_positions = np.stack([true_px, true_py], axis=1)

F = np.array([
    [1, 0, dt, 0],
    [0, 1, 0, dt],
    [0, 0, 1,  0],
    [0, 0, 0,  1],
])
H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
])


def generate_measurements(gps_noise_std, seed=42):
    """Simulate GPS observations z = true position + noise.
    Fixed seed so dragging the Q slider doesn't also change the noise pattern."""
    rng = np.random.default_rng(seed)
    return true_positions + rng.normal(0, gps_noise_std, true_positions.shape)


def run_kalman_filter(measurements, q, gps_noise_std):
    """Run one full predict -> update loop, returning the estimated position at each step."""
    Q = q * np.eye(4)
    R = (gps_noise_std ** 2) * np.eye(2)

    x_est = np.array([measurements[0, 0], measurements[0, 1], 0, 0])
    P = np.eye(4) * 10

    estimates = []
    for z in measurements:
        x_pred = F @ x_est
        P_pred = F @ P @ F.T + Q

        y = z - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)

        x_est = x_pred + K @ y
        P = (np.eye(4) - K @ H) @ P_pred
        estimates.append(x_est.copy())

    return np.array(estimates)


def rmse(a, b):
    return np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1)))


# ---------- Layout: main chart + two sliders + one button ----------
fig, ax = plt.subplots(figsize=(8, 7))
plt.subplots_adjust(bottom=0.28)

init_r, init_q = 0.8, 0.05
measurements = generate_measurements(init_r)
estimates = run_kalman_filter(measurements, init_q, init_r)

(line_true,) = ax.plot(true_positions[:, 0], true_positions[:, 1],
                        color="black", linewidth=2, label="Ground truth")
scatter_meas = ax.scatter(measurements[:, 0], measurements[:, 1],
                           color="red", s=10, alpha=0.4, label="Noisy GPS")
(line_est,) = ax.plot(estimates[:, 0], estimates[:, 1],
                       color="blue", linewidth=2, label="Kalman estimate")

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.axis("equal")
ax.grid(alpha=0.3)
ax.legend(loc="upper left")

title = ax.set_title("")


def update_title():
    rz = rmse(measurements, true_positions)
    rx = rmse(estimates, true_positions)
    reduction = (1 - rx / rz) * 100
    title.set_text(f"GPS RMSE {rz:.2f} m  |  Kalman RMSE {rx:.2f} m  |  error cut {reduction:.0f}%")


update_title()

# Slider: GPS noise R
ax_r = plt.axes([0.15, 0.14, 0.65, 0.03])
slider_r = Slider(ax_r, "GPS noise (R)", 0.1, 2.5, valinit=init_r, valstep=0.1)

# Slider: process noise Q
ax_q = plt.axes([0.15, 0.08, 0.65, 0.03])
slider_q = Slider(ax_q, "Process noise (Q)", 0.001, 0.5, valinit=init_q, valstep=0.001)

# Button: draw a fresh batch of noise
ax_btn = plt.axes([0.82, 0.02, 0.12, 0.04])
button_regen = Button(ax_btn, "Regenerate")


def redraw(_event=None, regenerate_noise=False):
    global measurements, estimates
    r_val = slider_r.val
    q_val = slider_q.val

    if regenerate_noise:
        # Bump the seed on each button click, i.e. draw a fresh batch of noise
        redraw.seed_counter += 1
        measurements = generate_measurements(r_val, seed=redraw.seed_counter)
    else:
        measurements = generate_measurements(r_val, seed=redraw.seed_counter)

    estimates = run_kalman_filter(measurements, q_val, r_val)

    scatter_meas.set_offsets(measurements)
    line_est.set_data(estimates[:, 0], estimates[:, 1])
    update_title()
    fig.canvas.draw_idle()


redraw.seed_counter = 42

slider_r.on_changed(redraw)
slider_q.on_changed(redraw)
button_regen.on_clicked(lambda event: redraw(event, regenerate_noise=True))

plt.show()