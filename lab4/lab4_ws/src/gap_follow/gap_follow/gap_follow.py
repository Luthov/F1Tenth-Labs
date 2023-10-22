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
        self.drive_msg.drive.speed = 0.0
        self.drive_msg.drive.steering_angle = 0.0

        self.ranges = None
        self.angle_increment = None
        self.angle_max = None
        self.angle_min = None
        
        self.distance_threshold = 3.0
        self.cone_size = 90 # -45 to 45 degrees
        self.car_size = 5 # metres
        self.cone_ranges = []
        self.no_indexes = 0

    def get_index(self, angle): # -ve angles are on the right side while +ve angles are on the left
        
        angle_rad = angle * (math.pi / 180)
        index = int( abs(self.angle_min - angle_rad) / self.angle_increment )

        return index

    # function name are usually verbs thanks 
    def cone_processing(self, cone_size):
        
        total_index = int((cone_size * (math.pi / 180)) / self.angle_increment)
        end = self.get_index(cone_size / 2) # Taking the middle as 0 degrees, outcome would be 45 degrees
        start = self.get_index(-cone_size / 2) # -45 degrees
        self.cone_ranges = self.ranges[start:end]

    # function name are usually verbs thanks 
    def bubble_diameter(self, distance):
        
        theta = self.car_size / distance
        self.no_indexes = int(theta / self.angle_increment)
        print(self.no_indexes, theta)


    def safety_bubble(self, car_size, threshold):
        
        # indexes = []
        # for i in range(len(ranges) - 1):
        #     delta_distance = ranges[i+1] - ranges[i]
        #     if delta_distance < threshold:
        #         indexes.append(i)hostname
        if self.cone_ranges:
            shortest_dist = min(self.cone_ranges)
            shortest_dist_ind = self.cone_ranges.index(shortest_dist) 
            
            self.bubble_diameter(shortest_dist)

            try:
                for i in range(self.no_indexes):
                    print("in for loop")
                    self.cone_ranges[int(shortest_dist_ind - (self.no_indexes / 2) + i)] = 0.0
                    print("Yay")
            except IndexError:
                print("Out of range dumbass")
    
    def get_direction(self):
        
        if self.cone_ranges:
            furthest_dist_ind = self.cone_ranges.index(max(self.cone_ranges))
            angle_size = furthest_dist_ind * self.angle_increment

            self.drive_msg.drive.steering_angle = angle_size - (self.cone_size * math.pi / 2 * 180)
            # if angle_size > midpoint:
            #     self.drive_msg.drive.steering_angle = angle_size - midpoint
            # else:
            #     self.drive_msg.drive.steering_angle = angle_size - midpoint
        

    def scan_callback(self, scan_data):
        
        self.ranges = scan_data.ranges
        self.angle_max = scan_data.angle_max
        self.angle_min = scan_data.angle_min
        self.angle_increment = scan_data.angle_increment

        self.cone_processing(self.cone_size)
        if self.cone_ranges: 
            self.safety_bubble(self.car_size, self.distance_threshold)
            self.get_direction()
            self.pub_drive.publish(self.drive_msg)
        

def main(args=None):

    rclpy.init(args=args)
    gap_follow = GapFollow()

    rclpy.spin(gap_follow)
    gap_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main ()
