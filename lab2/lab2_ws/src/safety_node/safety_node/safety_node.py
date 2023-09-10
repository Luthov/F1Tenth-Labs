import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import math

class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')
        
        # Subscribing to relevant topics
        self.sub_odom = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)

        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.pub_brake = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10)
        
        self.sub_scan
        self.sub_odom
        
        self.longitudinal_vel = 0
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = 0.0

    def scan_callback(self, scan_data):
        
        ranges = scan_data.ranges
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        threshold = 0.5
        angle = angle_min
        ttc = float('inf')
 
        for i in range(len(ranges)):
            # Fuck F1Tenth bunch of fkin liars
            r_dot = max(self.longitudinal_vel * math.cos(angle), 0)
            r = ranges[i]
            if (r_dot != 0) and (r != 0):
                ttc = r / r_dot
            angle += angle_increment
            cond = ttc <= threshold
            # self.get_logger().info(f"TTC: {ttc} , {cond}")
            if ttc <= threshold:
                self.pub_brake.publish(self.drive_msg)
                self.get_logger().info(f"Breaking Now")



    def odom_callback(self, odom_data):
        
        self.longitudinal_vel = odom_data.twist.twist.linear.x

        # self.get_logger().info(f"{self.longitudinal_vel}")

def main(args=None):
    
    rclpy.init(args=args)

    safety_node = SafetyNode()

    rclpy.spin(safety_node)

    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()

