#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):

    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        new_speed = msg.drive.speed * 3
        new_steering_angle = msg.drive.steering_angle * 3

        self.get_logger().info(f"New Speed: {new_speed} m/s, New Steering Angle: {new_steering_angle} rad")


def main(args=None):
    rclpy.init(args=args)

    relay = Relay()

    rclpy.spin(relay)

    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
