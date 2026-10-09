#!/usr/bin/env python3
"""PD go-to-goal controller for the Gazebo diff-drive robot (ported from the EN.605.613 midterm).

Reads the robot pose from the odom -> chassis transform (published by Gazebo's DiffDrive plugin
through the bridge), listens for goals on /goal_pose (RViz "2D Goal Pose"), and publishes
geometry_msgs/Twist on /cmd_vel.

Only change from the midterm version: output speeds are clamped, because a wheel radius of 0.4 m
in a real physics engine will happily flip or slip the robot at the speeds the raw law asks for.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def clamp(x, lim):
    return max(-lim, min(lim, x))


class DiffDrivePID(Node):

    def __init__(self):
        super().__init__('pid_controller')

        # PD gains (no integral term, as in the midterm).
        self.declare_parameter('kp_v', 1.0)
        self.declare_parameter('kd_v', 0.4)
        self.declare_parameter('kp_w', 2.0)
        self.declare_parameter('kd_w', 0.3)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('max_v', 1.0)      # m/s
        self.declare_parameter('max_w', 1.5)      # rad/s
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('robot_frame', 'chassis')

        self.Kp_v = self.get_parameter('kp_v').value
        self.Kd_v = self.get_parameter('kd_v').value
        self.Kp_w = self.get_parameter('kp_w').value
        self.Kd_w = self.get_parameter('kd_w').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.goal_x = None
        self.goal_y = None
        self.reached = False

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_time = None

        self.dt = 1.0 / 30.0
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('pid_controller started, listening on /goal_pose')

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.reached = False
        self.get_logger().info(f'New goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def control_loop(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, self.robot_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        theta = yaw_from_quaternion(tf.transform.rotation)
        now = self.get_clock().now()

        # Estimate the robot's own velocity via finite difference of pose.
        v_measured = 0.0
        w_measured = 0.0
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt > 0:
                vx = (x - self.prev_x) / dt
                vy = (y - self.prev_y) / dt
                v_measured = vx * math.cos(theta) + vy * math.sin(theta)
                w_measured = normalize_angle(theta - self.prev_theta) / dt

        self.prev_x, self.prev_y, self.prev_theta, self.prev_time = x, y, theta, now

        cmd = Twist()
        if self.goal_x is not None:
            dx = self.goal_x - x
            dy = self.goal_y - y
            distance = math.hypot(dx, dy)

            if distance < self.goal_tolerance:
                if not self.reached:
                    self.reached = True
                    self.get_logger().info('Goal reached')
            else:
                desired_heading = math.atan2(dy, dx)
                heading_error = normalize_angle(desired_heading - theta)

                v = self.Kp_v * distance * math.cos(heading_error) - self.Kd_v * v_measured
                w = self.Kp_w * heading_error - self.Kd_w * w_measured

                cmd.linear.x = clamp(v, self.max_v)
                cmd.angular.z = clamp(w, self.max_w)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
