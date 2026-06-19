import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class DroneSimulator(Node):
    def __init__(self):
        super().__init__('drone_simulator')

        # timestep
        self.dt = 0.02

        # state
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0
        self.yaw = 0.0

        # velocity state (important for realism)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # dynamics
        self.tau_xy = 0.6
        self.tau_z = 0.4

        # commands
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vz_cmd = 0.0
        self.wz_cmd = 0.0

        # ROS interfaces
        self.cmd_sub = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/drone/odom',
            10
        )

        # TF broadcaster (THIS FIXES YOUR PROBLEM)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Drone simulator started.')

    def cmd_callback(self, msg):
        self.vx_cmd = msg.linear.x
        self.vy_cmd = msg.linear.y
        self.vz_cmd = msg.linear.z
        self.wz_cmd = msg.angular.z

    def update(self):
        # first-order lag dynamics
        self.vx += (self.vx_cmd - self.vx) * self.dt / self.tau_xy
        self.vy += (self.vy_cmd - self.vy) * self.dt / self.tau_xy
        self.vz += (self.vz_cmd - self.vz) * self.dt / self.tau_z

        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt
        self.yaw += self.wz_cmd * self.dt

        # odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'drone_body'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz

        self.odom_pub.publish(odom)

        # TF (CRITICAL FOR RVIZ + URDF)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'drone_body'

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = self.z

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
