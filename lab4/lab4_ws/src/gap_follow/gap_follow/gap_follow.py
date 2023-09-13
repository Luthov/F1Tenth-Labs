import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import math

class GapFollow(Node):

    def __init__(self):
        super().__init__('gap_follow')

def main(args=None):

    rclpy.init(args=args)

    gap_follow = GapFollow()

    rclpy.spin(gap_follow)
    gap_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
