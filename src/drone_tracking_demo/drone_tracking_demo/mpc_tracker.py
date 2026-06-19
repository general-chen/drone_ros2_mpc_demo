import math
import csv
import os

import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from std_msgs.msg import Float32


class MPCTracker(Node):
    def __init__(self):
        super().__init__('mpc_tracker')

        self.dt = 0.1          # MPC control interval
        self.t = 0.0
        
        self.err_pub = self.create_publisher(
            Float32,
            '/drone/tracking_error',
            10
        )
        self.cumulative_abs_error_xy = 0.0

        # Current drone state
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0

        # Reference trajectory parameters
        self.radius = 2.0
        self.omega = 0.25
        self.z_ref = 1.0
        self.trajectory_mode = "circle"

        # MPC parameters
        self.N = 12            # prediction horizon
        self.max_v = 1.0

        self.q_pos = 12.0      # position tracking weight
        self.q_z = 5.0
        self.r_u = 0.15        # control effort weight
        self.r_du = 0.05       # command smoothness weight

        self.prev_u = np.zeros(3)

        self.odom_received = False

        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

        self.ref_marker_pub = self.create_publisher(
            Marker,
            '/drone/reference_marker',
            10
        )

        self.drone_marker_pub = self.create_publisher(
            Marker,
            '/drone/drone_marker',
            10
        )

        self.ref_path_pub = self.create_publisher(
            Path,
            '/drone/ref_path',
            10
        )

        self.actual_path_pub = self.create_publisher(
            Path,
            '/drone/actual_path',
            10
        )

        self.ref_path = Path()
        self.ref_path.header.frame_id = 'world'

        self.actual_path = Path()
        self.actual_path.header.frame_id = 'world'

        self.log_dir = os.path.expanduser('~/ros2_ws/drone_tracking_logs')
        os.makedirs(self.log_dir, exist_ok=True)

        self.log_path = os.path.join(self.log_dir, 'mpc_tracking_log.csv')
        self.log_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            'time',
            'x_ref', 'y_ref', 'z_ref',
            'x', 'y', 'z',
            'error_xy', 'error_z', 'error_3d',
            'cmd_vx', 'cmd_vy', 'cmd_vz',
            'solve_success', 'solve_cost',
            'error_x', 'error_y', 'error_z_signed',
            'cumulative_abs_error_xy'
        ])

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('MPC tracker started.')
        self.get_logger().info(f'Logging MPC data to: {self.log_path}')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.odom_received = True
            
    def reference(self, t):
        if self.trajectory_mode == "circle":
            x_ref = self.radius * math.cos(self.omega * t)
            y_ref = self.radius * math.sin(self.omega * t)
            z_ref = self.z_ref
        elif self.trajectory_mode == "figure8":
            x_ref = 2.0 * math.sin(0.25 * t)
            y_ref = 1.2 * math.sin(0.5 * t)
            z_ref = 1.0
        elif self.trajectory_mode == "random_smooth":
            x_ref = (
                2.0 * math.sin(0.25 * t) +
                1.2 * math.sin(0.11 * t + 1.7) +
                0.6 * math.sin(0.53 * t + 0.3)
            )
            y_ref = (
                2.0 * math.cos(0.22 * t) +
                1.0 * math.sin(0.17 * t + 2.1) +
                0.5 * math.cos(0.41 * t)
            )
            z_ref = 1.0 + 0.3 * math.sin(0.3 * t)
        else:
            raise ValueError(f'Unsupported trajectory mode: {self.trajectory_mode}')

        return np.array([x_ref, y_ref, z_ref], dtype=float)

    def predict_states(self, x0, u_sequence):
        states = []
        x = np.array(x0, dtype=float)

        for k in range(self.N):
            u = u_sequence[3 * k: 3 * k + 3]
            x = x + u * self.dt
            states.append(x.copy())

        return states

    def objective(self, u_sequence, x0):
        cost = 0.0
        states = self.predict_states(x0, u_sequence)

        last_u = self.prev_u.copy()

        for k, x_pred in enumerate(states):
            t_future = self.t + (k + 1) * self.dt
            ref = self.reference(t_future)

            pos_error = x_pred - ref

            cost += self.q_pos * (pos_error[0] ** 2 + pos_error[1] ** 2)
            cost += self.q_z * (pos_error[2] ** 2)

            u = u_sequence[3 * k: 3 * k + 3]
            cost += self.r_u * np.dot(u, u)

            du = u - last_u
            cost += self.r_du * np.dot(du, du)

            last_u = u.copy()

        return cost

    def solve_mpc(self):
        x0 = np.array([self.x, self.y, self.z], dtype=float)

        # Initial guess: repeat previous control over the horizon
        u0 = np.tile(self.prev_u, self.N)

        bounds = [(-self.max_v, self.max_v)] * (3 * self.N)

        result = minimize(
            self.objective,
            u0,
            args=(x0,),
            method='SLSQP',
            bounds=bounds,
            options={
                'maxiter': 40,
                'ftol': 1e-3,
                'disp': False
            }
        )

        if result.success:
            u_opt = result.x[0:3]
            solve_success = 1
            solve_cost = float(result.fun)
        else:
            # Fallback: safe proportional controller
            ref_now = self.reference(self.t)
            error = ref_now - x0
            u_opt = np.clip(0.8 * error, -self.max_v, self.max_v)
            solve_success = 0
            solve_cost = float(result.fun) if result.fun is not None else -1.0

        self.prev_u = u_opt.copy()

        return u_opt, solve_success, solve_cost

    def control_loop(self):
        if not self.odom_received:
            return

        self.t += self.dt

        ref_now = self.reference(self.t)
        x_ref, y_ref, z_ref = ref_now

        u_opt, solve_success, solve_cost = self.solve_mpc()

        cmd = Twist()
        cmd.linear.x = float(u_opt[0])
        cmd.linear.y = float(u_opt[1])
        cmd.linear.z = float(u_opt[2])
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        ex = x_ref - self.x
        ey = y_ref - self.y
        ez = z_ref - self.z

        error_xy = math.sqrt(ex**2 + ey**2)
        error_z = abs(ez)
        error_3d = math.sqrt(ex**2 + ey**2 + ez**2)
        self.cumulative_abs_error_xy += error_xy * self.dt

        error_msg = Float32()
        error_msg.data = float(error_xy)
        self.err_pub.publish(error_msg)

        self.csv_writer.writerow([
            self.t,
            x_ref, y_ref, z_ref,
            self.x, self.y, self.z,
            error_xy, error_z, error_3d,
            cmd.linear.x, cmd.linear.y, cmd.linear.z,
            solve_success, solve_cost,
            ex, ey, ez,
            self.cumulative_abs_error_xy
        ])
        self.log_file.flush()

        self.publish_paths(x_ref, y_ref, z_ref)
        self.publish_markers(x_ref, y_ref, z_ref)

    def publish_paths(self, x_ref, y_ref, z_ref):
        now = self.get_clock().now().to_msg()

        self.ref_path.header.stamp = now
        ref_pose = PoseStamped()
        ref_pose.header.stamp = now
        ref_pose.header.frame_id = 'world'
        ref_pose.pose.position.x = float(x_ref)
        ref_pose.pose.position.y = float(y_ref)
        ref_pose.pose.position.z = float(z_ref)
        self.ref_path.poses.append(ref_pose)

        self.actual_path.header.stamp = now
        actual_pose = PoseStamped()
        actual_pose.header.stamp = now
        actual_pose.header.frame_id = 'world'
        actual_pose.pose.position.x = self.x
        actual_pose.pose.position.y = self.y
        actual_pose.pose.position.z = self.z
        self.actual_path.poses.append(actual_pose)

        max_len = 1000
        self.ref_path.poses = self.ref_path.poses[-max_len:]
        self.actual_path.poses = self.actual_path.poses[-max_len:]

        self.ref_path_pub.publish(self.ref_path)
        self.actual_path_pub.publish(self.actual_path)

    def publish_markers(self, x_ref, y_ref, z_ref):
        now = self.get_clock().now().to_msg()

        ref_marker = Marker()
        ref_marker.header.frame_id = 'world'
        ref_marker.header.stamp = now
        ref_marker.ns = 'reference'
        ref_marker.id = 0
        ref_marker.type = Marker.SPHERE
        ref_marker.action = Marker.ADD
        ref_marker.pose.position.x = float(x_ref)
        ref_marker.pose.position.y = float(y_ref)
        ref_marker.pose.position.z = float(z_ref)
        ref_marker.scale.x = 0.18
        ref_marker.scale.y = 0.18
        ref_marker.scale.z = 0.18
        ref_marker.color.r = 1.0
        ref_marker.color.g = 0.2
        ref_marker.color.b = 0.2
        ref_marker.color.a = 1.0

        drone_marker = Marker()
        drone_marker.header.frame_id = 'world'
        drone_marker.header.stamp = now
        drone_marker.ns = 'drone'
        drone_marker.id = 1
        drone_marker.type = Marker.CUBE
        drone_marker.action = Marker.ADD
        drone_marker.pose.position.x = self.x
        drone_marker.pose.position.y = self.y
        drone_marker.pose.position.z = self.z
        drone_marker.scale.x = 0.30
        drone_marker.scale.y = 0.30
        drone_marker.scale.z = 0.10
        drone_marker.color.r = 0.2
        drone_marker.color.g = 0.4
        drone_marker.color.b = 1.0
        drone_marker.color.a = 1.0

        self.ref_marker_pub.publish(ref_marker)
        self.drone_marker_pub.publish(drone_marker)

    def destroy_node(self):
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
            self.get_logger().info(f'Saved MPC tracking log to: {self.log_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPCTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
