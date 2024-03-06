#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
# TODO CHECK: include needed ROS msg type headers and libraries
from visualization_msgs.msg import Marker

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.sub_marker = self.create_subscription(Marker, 'visualization_marker', self.waypoint_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, 'ego_racecar/odom', self.pose_callback, 10)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.pub_marker = self.create_publisher(Marker, 'goal_point', 10)

        self.lookahead_distance = 1.0
        self.prev_index = 0

        scale = 1.0
        self.goal_point_marker = Marker()
        self.goal_point_marker.header.frame_id =  "/map"
        self.goal_point_marker.ns = "goal_point"
        self.goal_point_marker.id = 1
        self.goal_point_marker.type = Marker.SPHERE  # Custom self.marker type
        self.goal_point_marker.action = Marker.ADD # coordinate_arr        
        self.goal_point_marker.scale.x = scale
        self.goal_point_marker.scale.y = scale
        self.goal_point_marker.scale.z = scale
        self.goal_point_marker.color.a = 1.0
        self.goal_point_marker.color.r = 0.0
        self.goal_point_marker.color.g = 1.0
        self.goal_point_marker.color.b = 0.0


    def waypoint_callback(self, marker_data):
        self.waypoints = marker_data.points

    def pose_callback(self, odom_data):
        try:
            # TODO: find the current waypoint to track using methods mentioned in lecture

            coincident = False
            drive_data = AckermannDriveStamped()
            index = 0
            count = 0

            for i in range(len(self.waypoints)):

                waypoint = self.waypoints[i]
                waypoint_distance = round(np.sqrt((waypoint.x - odom_data.pose.pose.position.x)**2 + (waypoint.y - odom_data.pose.pose.position.y)**2), 1)

                if (waypoint_distance == self.lookahead_distance) and (index >= self.prev_index):
                    index = i
                    count += 1
                    if count == 2:
                        self.goal_point = self.waypoints[index]
                        coincident = True
                        print("coincident")
                        break

                elif (self.lookahead_distance - 0.5 < waypoint_distance < self.lookahead_distance + 0.5) and (index >= self.prev_index):
                    index = i
                    break

                self.prev_index = index
                    
            print(f"{index} | {self.prev_index}")
            if not coincident:
                self.goal_point = self.waypoints[5 + index]

            # print(f"Goal Point: {self.goal_point.x}, {self.goal_point.y}")

            
            # TODO: transform goal point to vehicle frame of reference [x, y]
            # DOES NOT WORK
            goal_point_vehicle_frame = [self.goal_point.x - odom_data.pose.pose.position.x, self.goal_point.y - odom_data.pose.pose.position.y]

            # TODO: calculate curvature/steering angle
            curvature = 2*goal_point_vehicle_frame[1]/self.lookahead_distance**2

            # TODO: publish drive message, don't forget to limit the steering angle.
            steering_angle = 100.0 * curvature
            drive_data.drive.steering_angle = steering_angle 
            drive_data.drive.speed = 1.0 # 0.1 * (1 / 1.2)**(steering_angle- 15)

            self.pub_drive.publish(drive_data)
            # print(steering_angle, drive_data.drive.speed)

            self.goal_point_marker.pose.position.x = self.goal_point.x # goal_point_vehicle_frame[0]
            self.goal_point_marker.pose.position.y = self.goal_point.y # goal_point_vehicle_frame[1] 
            self.goal_point_marker.pose.position.z = 0.0
            self.pub_marker.publish(self.goal_point_marker)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
