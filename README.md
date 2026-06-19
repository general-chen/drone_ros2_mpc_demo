# ROS2 UAV Trajectory Tracking Demo (P vs MPC)

This project demonstrates a ROS2-based UAV simulation environment with trajectory tracking control. It compares a simple proportional controller (P) and a Model Predictive Controller (MPC) for following a circular reference path.

---

## Features

- ROS2-based UAV simulator
- Odometry + TF broadcasting
- RViz visualization (drone + reference path + actual path)
- P controller baseline
- MPC trajectory tracking controller (optimization-based)
- Real-time performance logging (CSV)

---

## Demo Overview

The system consists of:

- Drone simulator node (kinematic UAV model)
- MPC tracker node (trajectory optimization)
- RViz visualization (TF + paths + markers)

The UAV follows a circular trajectory in 3D space while comparing control performance.

---

## Control Methods

### P Controller
- Reactive control based on current position error
- Simple proportional velocity command

### MPC Controller
- Predicts future trajectory (horizon-based optimization)
- Minimizes tracking error + control effort
- Produces smoother and more stable tracking

---

## Visualization

In RViz:

- Green line → reference trajectory
- Blue line → actual UAV trajectory
- Marker → UAV position
- TF → coordinate frames

---

## How to Run

```bash
colcon build
source install/setup.bash
ros2 launch drone_tracking_demo full_demo.launch.py
