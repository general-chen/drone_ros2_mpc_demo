import math
import csv
import os


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        self.dt = 0.02
        self.t = 0.0

        self.log_dir = os.path.expanduser('~/ros2_ws/drone_tracking_logs')
        os.makedirs(self.log_dir, exist_ok=True)

        self.log_path = os.path.join(self.log_dir, 'p_controller_tracking_log.csv')
        self.log_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            'time',
            'x_ref', 'y_ref', 'z_ref',
            'x', 'y', 'z',
            'error_xy', 'error_z', 'error_3d',
            'cmd_vx', 'cmd_vy', 'cmd_vz'
        ])

        self.get_logger().info(f'Logging tracking data to: {self.log_path}')

        self.x = 0.0
        self.y = 0.0
        self.z = 1.0

        self.kp_xy = 1.2
        self.kp_z = 1.0
        self.max_v = 1.0

        self.radius = 2.0
        self.omega = 0.25
        self.z_ref = 1.0

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

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Path tracker started.')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.odom_received = True

    def reference(self, t):
        # Circular reference path
        x_ref = self.radius * math.cos(self.omega * t)
        y_ref = self.radius * math.sin(self.omega * t)
        z_ref = self.z_ref

        vx_ref = -self.radius * self.omega * math.sin(self.omega * t)
        vy_ref = self.radius * self.omega * math.cos(self.omega * t)
        vz_ref = 0.0

        return x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref

    def saturate(self, value, limit):
        return max(min(value, limit), -limit)

    def control_loop(self):
        if not self.odom_received:
            return

        self.t += self.dt

        x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref = self.reference(self.t)

        ex = x_ref - self.x
        ey = y_ref - self.y
        ez = z_ref - self.z

        cmd = Twist()
        cmd.linear.x = self.saturate(vx_ref + self.kp_xy * ex, self.max_v)
        cmd.linear.y = self.saturate(vy_ref + self.kp_xy * ey, self.max_v)
        cmd.linear.z = self.saturate(vz_ref + self.kp_z * ez, self.max_v)
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        error_xy = math.sqrt(ex**2 + ey**2)
        error_z = abs(ez)
        error_3d = math.sqrt(ex**2 + ey**2 + ez**2)

        self.csv_writer.writerow([
            self.t,
            x_ref, y_ref, z_ref,
            self.x, self.y, self.z,
            error_xy, error_z, error_3d,
            cmd.linear.x, cmd.linear.y, cmd.linear.z
        ])

        self.publish_paths(x_ref, y_ref, z_ref)
        self.publish_markers(x_ref, y_ref, z_ref)

    def publish_paths(self, x_ref, y_ref, z_ref):
        now = self.get_clock().now().to_msg()

        self.ref_path.header.stamp = now
        ref_pose = PoseStamped()
        ref_pose.header.stamp = now
        ref_pose.header.frame_id = 'world'
        ref_pose.pose.position.x = x_ref
        ref_pose.pose.position.y = y_ref
        ref_pose.pose.position.z = z_ref
        self.ref_path.poses.append(ref_pose)

        self.actual_path.header.stamp = now
        actual_pose = PoseStamped()
        actual_pose.header.stamp = now
        actual_pose.header.frame_id = 'world'
        actual_pose.pose.position.x = self.x
        actual_pose.pose.position.y = self.y
        actual_pose.pose.position.z = self.z
        self.actual_path.poses.append(actual_pose)

        # Keep path length reasonable
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
        ref_marker.pose.position.x = x_ref
        ref_marker.pose.position.y = y_ref
        ref_marker.pose.position.z = z_ref
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
            self.get_logger().info(f'Saved tracking log to: {self.log_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
