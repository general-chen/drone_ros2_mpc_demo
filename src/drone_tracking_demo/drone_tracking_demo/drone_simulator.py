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

        # Wind disturbance model
        self.declare_parameter("wind_level", "strong")
        self.wind_level = self.get_parameter("wind_level").get_parameter_value().string_value
        self.wind_scale_factors = {
            "none": 0.0,
            "mild": 0.5,
            "moderate": 1.0,
            "strong": 2.0,
            "extreme": 3.0,
        }
        if self.wind_level not in self.wind_scale_factors:
            self.get_logger().warning(
                f'Invalid wind_level "{self.wind_level}". Falling back to "strong".'
            )
            self.wind_level = "strong"
        self.get_logger().info(f'Wind level set to: {self.wind_level}')
        self.enable_burst_gust = True

        # Constant background wind velocity disturbance
        self.wind_bias_x = 0.15
        self.wind_bias_y = -0.10
        self.wind_bias_z = 0.00

        # Time-varying gust disturbance
        self.wind_amp_x = 0.25
        self.wind_amp_y = 0.20
        self.wind_amp_z = 0.05

        self.sim_time = 0.0
        self.last_wind_log_time = -999.0

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

    def compute_wind_disturbance(self, t):
        scale = self.wind_scale_factors.get(self.wind_level, self.wind_scale_factors["strong"])

        if scale == 0.0:
            return 0.0, 0.0, 0.0

        wind_vx = self.wind_bias_x + self.wind_amp_x * math.sin(0.7 * t)
        wind_vy = self.wind_bias_y + self.wind_amp_y * math.sin(0.5 * t + 1.2)
        wind_vz = self.wind_bias_z + self.wind_amp_z * math.sin(0.9 * t + 0.4)

        if self.enable_burst_gust and 12.0 <= t <= 18.0:
            wind_vx += 0.40
            wind_vy += -0.30
            wind_vz += 0.08

        return scale * wind_vx, scale * wind_vy, scale * wind_vz

    def update(self):
        # first-order lag dynamics
        self.vx += (self.vx_cmd - self.vx) * self.dt / self.tau_xy
        self.vy += (self.vy_cmd - self.vy) * self.dt / self.tau_xy
        self.vz += (self.vz_cmd - self.vz) * self.dt / self.tau_z

        wind_vx, wind_vy, wind_vz = self.compute_wind_disturbance(self.sim_time)
        vx_ground = self.vx + wind_vx
        vy_ground = self.vy + wind_vy
        vz_ground = self.vz + wind_vz

        self.x += vx_ground * self.dt
        self.y += vy_ground * self.dt
        self.z += vz_ground * self.dt
        self.yaw += self.wz_cmd * self.dt
        self.sim_time += self.dt

        if self.sim_time - self.last_wind_log_time >= 2.0:
            self.get_logger().info(
                f'Wind level: {self.wind_level} | vx={wind_vx:.3f}, vy={wind_vy:.3f}, vz={wind_vz:.3f}'
            )
            self.last_wind_log_time = self.sim_time

        # odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'drone_body'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        odom.twist.twist.linear.x = vx_ground
        odom.twist.twist.linear.y = vy_ground
        odom.twist.twist.linear.z = vz_ground

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
