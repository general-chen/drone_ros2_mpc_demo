import pandas as pd
import matplotlib.pyplot as plt

# Load data
file_path = "~/ros2_ws/drone_tracking_logs/p_controller_tracking_log.csv"
df = pd.read_csv(file_path)

# Fix path expansion
df["time"] = df["time"] - df["time"].iloc[0]

# ---------------------------
# 1. XY trajectory plot
# ---------------------------
plt.figure()
plt.plot(df["x_ref"], df["y_ref"], "g--", label="Reference")
plt.plot(df["x"], df["y"], "b", label="Actual")
plt.axis("equal")
plt.legend()
plt.title("Trajectory Tracking (ROS 2 P Controller)")
plt.xlabel("X")
plt.ylabel("Y")

# ---------------------------
# 2. Tracking error
# ---------------------------
plt.figure()
plt.plot(df["time"], df["error_xy"])
plt.title("Tracking Error (XY)")
plt.xlabel("Time (s)")
plt.ylabel("Error")

# ---------------------------
# 3. Control effort
# ---------------------------
plt.figure()
plt.plot(df["time"], df["cmd_vx"], label="vx")
plt.plot(df["time"], df["cmd_vy"], label="vy")
plt.legend()
plt.title("Control Commands")
plt.xlabel("Time (s)")
plt.ylabel("Velocity")

plt.show()
