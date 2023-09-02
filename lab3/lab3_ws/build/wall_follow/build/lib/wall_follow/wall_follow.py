import rclpy
import pandas
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import math

class WallFollow(Node):

    def __init__(self):
        super().__init__('wall_follow')

        # Subscribing to relevant topics
        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        self.pub_drive = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10)

        self.sub_scan

        self.drive_msg = AckermannDriveStamped()
        
        self.Kp = 1.0
        self.Ki = 0.001
        self.Kd = 0.0001
        # Something wrong with integral
        self.integral = 0.0
        self.prev_error_1 = 0.0
        self.prev_secs = 0.0
        self.prev_nsecs = 0.0

        self.log = {"secs": [], "nsecs": [], "actual_distance": []}

    def getRange(self, scan_data, angle):
        ranges = scan_data.ranges
        angle_rad = angle * (math.pi / 180)
        index = int( (angle_rad) / scan_data.angle_increment )
        return ranges[index]

    def scan_callback(self, scan_data):

        ranges = scan_data.ranges
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        angle_max = scan_data.angle_max

        secs = scan_data.header.stamp.sec
        nsecs = scan_data.header.stamp.nanosec 

        theta = 10 * (math.pi / 180)
        
        # 90 Degrees to the car
        distance_b = self.getRange(scan_data, 90) # ranges[901]
        # ~ 35 Degrees to the first scan
        distance_a = self.getRange(scan_data, 80) # ranges[760]
        
        alpha = math.atan( (distance_a * math.cos(theta) - distance_b) / (distance_a * math.sin(theta)) )

        actual_distance = distance_b * math.cos(alpha)
        desired_distance = 1.2 # Metres

        error = desired_distance - actual_distance
        lookahead_distance = 0.3 # Metres
        error_1 = error + lookahead_distance * math.sin(alpha)
        
        if (self.prev_secs == 0.0) & (self.prev_nsecs == 0.0) & (self.prev_error_1 == 0.0):
            self.prev_secs = secs
            self.prev_nsecs = nsecs
            self.prev_error_1 = error_1

        dt = secs - self.prev_secs + (nsecs - self.prev_nsecs) * 1e-9
        
        
        if dt != 0:
            self.integral += error_1 * dt
            steering_angle = 1 * ( (self.Kp * error_1) + (self.Ki * self.integral) + (self.Kd * (error_1 - self.prev_error_1) / dt) )
            self.drive_msg.drive.steering_angle = steering_angle
            steering_angle_degrees = abs(steering_angle * (180 / math.pi))

            self.prev_error_1 = error_1
            self.prev_secs = secs
            self.prev_nsecs = nsecs
            
            if (steering_angle_degrees > 0.0) & (steering_angle_degrees < 10.0):
                self.drive_msg.drive.speed = 1.5
            elif (steering_angle_degrees > 10.0) & (steering_angle_degrees < 20.0):
                self.drive_msg.drive.speed = 1.0
            else:
                self.drive_msg.drive.speed = 0.5
            
            self.pub_drive.publish(self.drive_msg)
            self.get_logger().info(f"alpha: {alpha:.2f} | Actual Distance: {actual_distance:.2f} | Steering Angle: {steering_angle_degrees:.2f} | dt: {dt:.2f}")
        
        #for i in range(len(ranges)):
        #    if 
            self.log["secs"].append(secs)
            self.log["nsecs"].append(nsecs)
            self.log["actual_distance"].append(actual_distance)

            #df = pandas.DataFrame(self.log)
            #df.to_csv("/root/logfile.csv", index=False)


def main(args=None):

    rclpy.init(args=args)

    wall_follow = WallFollow()

    rclpy.spin(wall_follow)
    print("hello")
    wall_follow.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':

    main()
