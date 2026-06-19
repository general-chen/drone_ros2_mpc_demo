import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

log_dir = os.path.expanduser("~/ros2_ws/drone_tracking_logs")

p_file = os.path.join(log_dir, "p_controller_tracking_log.csv")
mpc_file = os.path.join(log_dir, "mpc_tracking_log.csv")

p = pd.read_csv(p_file)
m = pd.read_csv(mpc_file)

p["time"] = p["time"] - p["time"].iloc[0]
m["time"] = m["time"] - m["time"].iloc[0]

p_rmse = np.sqrt(np.mean(p["error_xy"] ** 2))
m_rmse = np.sqrt(np.mean(m["error_xy"] ** 2))

p_mean = np.mean(p["error_xy"])
m_mean = np.mean(m["error_xy"])

p_max = np.max(p["error_xy"])
m_max = np.max(m["error_xy"])

# Ignore first 5 seconds to evaluate steady-state tracking.
p_ss = p[p["time"] > 5.0]
m_ss = m[m["time"] > 5.0]

p_ss_rmse = np.sqrt(np.mean(p_ss["error_xy"] ** 2))
m_ss_rmse = np.sqrt(np.mean(m_ss["error_xy"] ** 2))

p_ss_mean = np.mean(p_ss["error_xy"])
m_ss_mean = np.mean(m_ss["error_xy"])

print("P controller:")
print(f"  Mean XY error = {p_mean:.4f}")
print(f"  RMSE XY error = {p_rmse:.4f}")

print("MPC controller:")
print(f"  Mean XY error = {m_mean:.4f}")
print(f"  RMSE XY error = {m_rmse:.4f}")

# ---------------------------
# 1. XY trajectory comparison
# ---------------------------
plt.figure(figsize=(7, 6))
plt.plot(p["x_ref"], p["y_ref"], "k--", linewidth=2, label="Reference")
plt.plot(p["x"], p["y"], linewidth=2, label=f"P controller, RMSE={p_rmse:.3f}")
plt.plot(m["x"], m["y"], linewidth=2, label=f"MPC, RMSE={m_rmse:.3f}")
plt.axis("equal")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("ROS 2 Drone Path Tracking: P Controller vs MPC")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(log_dir, "trajectory_comparison.png"), dpi=300)

# ---------------------------
# 2. Tracking error comparison
# ---------------------------
plt.figure(figsize=(8, 4))
plt.plot(p["time"], p["error_xy"], linewidth=2, label="P controller")
plt.plot(m["time"], m["error_xy"], linewidth=2, label="MPC")
plt.xlabel("Time (s)")
plt.ylabel("XY tracking error (m)")
plt.title("Tracking Error Comparison")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(log_dir, "tracking_error_comparison.png"), dpi=300)

# ---------------------------
# 3. Command comparison
# ---------------------------
plt.figure(figsize=(8, 4))
plt.plot(p["time"], p["cmd_vx"], linewidth=1.5, label="P vx")
plt.plot(p["time"], p["cmd_vy"], linewidth=1.5, label="P vy")
plt.plot(m["time"], m["cmd_vx"], "--", linewidth=1.5, label="MPC vx")
plt.plot(m["time"], m["cmd_vy"], "--", linewidth=1.5, label="MPC vy")
plt.xlabel("Time (s)")
plt.ylabel("Velocity command (m/s)")
plt.title("Command Velocity Comparison")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(log_dir, "command_comparison.png"), dpi=300)

print("Maximum error:")
print(f"  P max XY error = {p_max:.4f}")
print(f"  MPC max XY error = {m_max:.4f}")

print("Steady-state after 5 s:")
print(f"  P mean XY error = {p_ss_mean:.4f}")
print(f"  P RMSE XY error = {p_ss_rmse:.4f}")
print(f"  MPC mean XY error = {m_ss_mean:.4f}")
print(f"  MPC RMSE XY error = {m_ss_rmse:.4f}")

# ---------------------------
# 4. Steady-state XY trajectory comparison after 5 s
# ---------------------------
plt.figure(figsize=(7, 6))
plt.plot(p_ss["x_ref"], p_ss["y_ref"], "k--", linewidth=2, label="Reference")
plt.plot(p_ss["x"], p_ss["y"], linewidth=2, label=f"P controller, RMSE={p_ss_rmse:.3f} m")
plt.plot(m_ss["x"], m_ss["y"], linewidth=2, label=f"MPC, RMSE={m_ss_rmse:.3f} m")
plt.axis("equal")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Steady-State Trajectory Tracking After 5 s")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(log_dir, "steady_state_trajectory_comparison.png"), dpi=300)

# ---------------------------
# 5. Steady-state tracking error comparison after 5 s
# ---------------------------
plt.figure(figsize=(8, 4))
plt.plot(p_ss["time"], p_ss["error_xy"], linewidth=2, label="P controller")
plt.plot(m_ss["time"], m_ss["error_xy"], linewidth=2, label="MPC")
plt.xlabel("Time (s)")
plt.ylabel("XY tracking error (m)")
plt.title("Steady-State Tracking Error After 5 s")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(log_dir, "steady_state_tracking_error_comparison.png"), dpi=300)

plt.show()
