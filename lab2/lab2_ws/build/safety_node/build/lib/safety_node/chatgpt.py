import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import math

def calculate_ttc(ranges, angles, v_longitudinal):
    ttc = float('inf')
    for i in range(len(ranges)):
        if ranges[i] == float('inf'):
            continue
        theta = angles[i]
        d = ranges[i]
        if theta == 0:
            continue
        v_lateral = v_longitudinal * math.tan(theta)
        if v_lateral == 0:
            continue
        ttc_i = d / v_lateral
        if ttc_i < ttc:
            ttc = ttc_i
    return ttc

class AutoBrake(Node):

    v_longitudinal = Odometry().twist.twist.linear.x

    def __init__(self):
        super().__init__('safety_node')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10
        )
        self.publisher_brake = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10
        )
        self.subscription_scan  # prevent unused variable warning
        self.subscription_odom

    def odom_callback(self, odom_msg):

        self.v_longitudinal = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
                
        ranges = scan_msg.ranges

        angles = []
        min_angle = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        for i in range(len(ranges)):
            angle = min_angle + i * angle_increment
            angles.append(angle)

        ttc = calculate_ttc(ranges, angles, self.v_longitudinal)
        #if ttc > 0:
        self.get_logger().info("" + str(ttc))
        if ttc < -500.0:
            ackermann_msg = AckermannDriveStamped()
            ackermann_msg.drive.speed = 0.0
            self.publisher_brake.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)

    auto_brake = AutoBrake()

    rclpy.spin(auto_brake)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_brake.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
