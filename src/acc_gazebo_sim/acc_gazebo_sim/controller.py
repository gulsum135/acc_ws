#!/usr/bin/env python3
"""Front-LiDAR ACC/AEB controller for a differential-drive Gazebo vehicle."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class GazeboAccController(Node):
    def __init__(self):
        super().__init__('gazebo_acc_controller')
        self.declare_parameter('target_speed_mps', 5.0)
        self.declare_parameter('stop_distance_m', 3.5)
        self.declare_parameter('critical_distance_m', 6.0)
        self.declare_parameter('comfort_decel_mps2', 1.5)
        self.declare_parameter('emergency_decel_mps2', 5.0)
        self.declare_parameter('max_accel_mps2', 1.0)
        self.declare_parameter('front_angle_deg', 18.0)
        self.declare_parameter('scan_timeout_sec', 0.75)

        self.target_speed = float(self.get_parameter('target_speed_mps').value)
        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.critical_distance = float(self.get_parameter('critical_distance_m').value)
        self.comfort_decel = float(self.get_parameter('comfort_decel_mps2').value)
        self.emergency_decel = float(self.get_parameter('emergency_decel_mps2').value)
        self.max_accel = float(self.get_parameter('max_accel_mps2').value)
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)

        self.speed = 0.0
        self.front_distance = None
        self.last_scan_time = None
        self.last_tick = self.get_clock().now()

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/sim/state', 10)
        self.create_timer(0.05, self.control_tick)
        self.get_logger().info('Gazebo ACC/AEB controller started; waiting for /scan and /odom.')

    def scan_callback(self, msg):
        values = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if abs(angle) <= self.front_angle:
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    values.append(distance)
            angle += msg.angle_increment
        self.front_distance = min(values) if values else None
        self.last_scan_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.speed = max(0.0, float(msg.twist.twist.linear.x))

    def control_tick(self):
        now = self.get_clock().now()
        dt = max(0.001, min(0.2, (now - self.last_tick).nanoseconds / 1e9))
        self.last_tick = now

        state = 'SEYIR'
        scan_fresh = (
            self.last_scan_time is not None
            and (now - self.last_scan_time).nanoseconds / 1e9 <= self.scan_timeout
        )

        if not scan_fresh or self.front_distance is None:
            desired_speed = 0.0
            state = 'SENSOR_BEKLENIYOR' if not scan_fresh else 'DURDU'
        else:
            gap = self.front_distance
            usable_gap = max(0.0, gap - self.stop_distance)
            safe_speed = math.sqrt(2.0 * self.comfort_decel * usable_gap)
            desired_speed = min(self.target_speed, safe_speed)

            if gap <= self.critical_distance and self.speed > desired_speed + 0.25:
                desired_speed = max(0.0, self.speed - self.emergency_decel * dt)
                state = 'ACIL_FREN'
            elif self.speed > desired_speed + 0.1:
                state = 'YAVASLIYOR'
            elif desired_speed <= 0.05 and self.speed <= 0.08:
                state = 'DURDU'
            elif self.speed < desired_speed - 0.1:
                state = 'HIZLANIYOR'

        if state == 'ACIL_FREN':
            next_speed = desired_speed
        elif self.speed > desired_speed:
            next_speed = max(desired_speed, self.speed - self.comfort_decel * dt)
        else:
            next_speed = min(desired_speed, self.speed + self.max_accel * dt)

        command = Twist()
        command.linear.x = max(0.0, next_speed)
        self.cmd_pub.publish(command)
        self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = GazeboAccController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
