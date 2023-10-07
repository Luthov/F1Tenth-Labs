import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import math

class GapFollow(Node):
    def __init__(self):
        super().__init__('gap_follow')

        # Subscribing to topics
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
        
        self.distance_threshold = 3.0
        self.cone_size = 90 # -45 to 45 degrees
        self.car_size = 5 # metres

    def get_index(self, scan_data, angle):
        ranges = scan_data.ranges
        angle_rad = angle * (math.pi / 180)
        index = int( abs(angle_rad - scan_data.angle_max) / scan_data.angle_increment )

        return index

    # function name are usually verbs thanks 
    def cone_processing(self, scan_data, cone_size):
        
        total_index = int((cone_size * (math.pi / 180)) / scan_data.angle_increment)
        end = self.get_index(scan_data, (cone_size / 2))
        start = end - total_index
        cone_range = scan_data.ranges[start:end]
        return cone_range

    # function name are usually verbs thanks 
    def bubble_diameter(self, distance, angle_increment):
        
        theta = self.car_size / distance
        no_indexes = theta / angle_increment

        return no_indexes

    def safety_bubble(self, ranges, car_size, threshold, angle_increment):
        
        # indexes = []
        # for i in range(len(ranges) - 1):
        #     delta_distance = ranges[i+1] - ranges[i]
        #     if delta_distance < threshold:
        #         indexes.append(i)hostname
        if not ranges == None:
            shortest_dist = min(ranges)
            shortest_dist_ind = ranges.index(shortest_dist) 

            no_indexes = self.bubble_diameter(shortest_dist, angle_increment)
            for i in range(len(no_indexes)):
                ranges[shortest_dist_ind - (no_indexes / 2) + i] = 0.0
    
    def get_direction(self, ranges, angle_increment):
        
        if not ranges == None:
            furthest_dist_ind = ranges.index(max(ranges))
            steering_angle = furthest_dist_ind * angle_increment

        return steering_angle

    def scan_callback(self, scan_data):
        
        ranges = scan_data.ranges
        angle_increment = scan_data.angle_increment

        cone_ranges = self.cone_processing(scan_data, self.cone_size)
        self.safety_bubble(cone_ranges, self.car_size, self.distance_threshold, angle_increment)
        steering_angle = self.get_direction(cone_ranges, angle_increment)


def main(args=None):

    rclpy.init(args=args)
    gap_follow = GapFollow()

    rclpy.spin(gap_follow)
    gap_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main ()
