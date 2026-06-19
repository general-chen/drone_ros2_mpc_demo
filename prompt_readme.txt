You are working in a ROS 2 Jazzy Python package repository:

~/ros2_ws

Current branch:
wind-disturbance-module

Task:
Create a professional README.md for the ROS 2 drone MPC tracking demo.

The repository contains:
- ROS 2 Jazzy Python package: src/drone_tracking_demo
- Drone simulator with first-order velocity dynamics
- MPC trajectory tracker
- P-controller tracker
- URDF quadrotor model
- RViz visualization
- TF world -> drone_body
- wind disturbance module
- real-time tracking-error topic
- CSV logging
- wind/no-wind metrics analysis
- launch-configurable wind_level
- launch-configurable trajectory_mode

README.md should include:

1. Title:
   ROS 2 Drone MPC Tracking Demo under Wind Disturbance

2. Short project description:
   This project demonstrates a ROS 2-based UAV trajectory tracking system using MPC, simulated drone dynamics, URDF/RViz visualization, and wind disturbance robustness evaluation.

3. Main features:
   - ROS 2 Jazzy multi-node system
   - simulated drone dynamics
   - MPC tracking controller
   - P-controller baseline
   - URDF quadrotor model
   - TF and RViz visualization
   - wind disturbance levels
   - selectable trajectory modes
   - tracking-error topic
   - CSV logging and metric analysis

4. System architecture diagram in ASCII:
   trajectory reference -> MPC tracker -> /drone/cmd_vel -> simulator -> /drone/odom + TF -> RViz

5. ROS topics:
   - /drone/cmd_vel
   - /drone/odom
   - /drone/ref_path
   - /drone/actual_path
   - /drone/reference_marker
   - /drone/tracking_error
   - /tf
   - /robot_description

6. Requirements:
   - Ubuntu 24.04
   - ROS 2 Jazzy
   - Python 3
   - scipy
   - numpy
   - pandas
   - matplotlib
   - robot_state_publisher
   - rviz2

7. Build instructions:
   cd ~/ros2_ws
   colcon build --packages-select drone_tracking_demo
   source install/setup.bash

8. Run commands:
   no wind circle:
   ros2 launch drone_tracking_demo full_demo.launch.py wind_level:=none trajectory_mode:=circle

   strong wind circle:
   ros2 launch drone_tracking_demo full_demo.launch.py wind_level:=strong trajectory_mode:=circle

   strong wind random smooth:
   ros2 launch drone_tracking_demo full_demo.launch.py wind_level:=strong trajectory_mode:=random_smooth

9. Available wind levels:
   - none
   - mild
   - moderate
   - strong
   - extreme

10. Available trajectory modes:
   - circle
   - figure8
   - random_smooth

11. Metrics and analysis:
   Explain that the MPC tracker publishes /drone/tracking_error and saves CSV logs to:
   ~/ros2_ws/drone_tracking_logs/mpc_tracking_log.csv

   Explain how to save no-wind and strong-wind logs:
   cp ~/ros2_ws/drone_tracking_logs/mpc_tracking_log.csv ~/ros2_ws/drone_tracking_logs/mpc_tracking_log_no_wind.csv
   cp ~/ros2_ws/drone_tracking_logs/mpc_tracking_log.csv ~/ros2_ws/drone_tracking_logs/mpc_tracking_log_wind_strong.csv

   Run:
   python3 analyze_wind_metrics.py

12. Example metrics:
   Include these values:
   - No-wind steady-state RMSE after 5 s: 0.0158 m
   - Strong-wind steady-state RMSE after 5 s: 0.2948 m
   - No-wind gust-window RMSE from 12–18 s: 0.0122 m
   - Strong-wind gust-window RMSE from 12–18 s: 0.4001 m
   - Gust-window RMSE increase: about 32.8x

13. Suggested demo video sequence:
   - no-wind circle tracking
   - strong-wind circle tracking
   - random_smooth trajectory under strong wind
   - metric plot comparison

14. Future work:
   - PX4/Gazebo integration
   - obstacle avoidance
   - visual tracking
   - RL-based controller
   - real UAV deployment

15. Add a note:
   This is a compact ROS 2 learning and demonstration project, not a high-fidelity aerodynamic UAV simulator.

Do not modify code files.
Only create or update README.md.

Return a concise summary.
