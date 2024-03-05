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
        self.threshold = 0.1
        self.prev_waypoint_distance = 0.0
        self.waypoints = None

        scale = 1.0
        self.goal_point_marker = Marker()
        self.goal_point_marker.header.frame_id = "ego_racecar/base_link"
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
        if self.waypoints is None:
            pass
        else:
            # TODO: find the current waypoint to track using methods mentioned in lecture
            # Calculate the distance from the current pose to each waypoint and if lookahead distance is in between previous distance and current distance interpolate else 
            coincident = False
            prev_delta = float('inf')
            drive_data = AckermannDriveStamped()

            for i in range(len(self.waypoints)):

                waypoint = self.waypoints[i]
                waypoint_distance = np.sqrt((waypoint.x - odom_data.pose.pose.position.x)**2 + (waypoint.y - odom_data.pose.pose.position.y)**2)
                delta = abs(waypoint_distance - self.lookahead_distance)


                if waypoint_distance == self.lookahead_distance:
                    self.goal_point = waypoint
                    coincident = True
                    break

                if min(prev_delta, delta) == delta:
                    self.index = i
                    
            if not coincident:
                self.goal_point = self.waypoints[self.index]

            prev_delta = delta
            
            # TODO: transform goal point to vehicle frame of reference [x, y]
            goal_point_vehicle_frame = [self.goal_point.x - odom_data.pose.pose.position.x, self.goal_point.y - odom_data.pose.pose.position.y]

            # TODO: calculate curvature/steering angle
            curvature = 2*goal_point_vehicle_frame[1]/self.lookahead_distance**2

            # TODO: publish drive message, don't forget to limit the steering angle.
            steering_angle = 0.01 * curvature
            drive_data.drive.steering_angle = steering_angle 
            drive_data.drive.speed = 0.2 * (1 / 1.2)**(steering_angle- 15)

            self.pub_drive.publish(drive_data)

            self.goal_point_marker.pose.position.x = self.goal_point.x
            self.goal_point_marker.pose.position.y = self.goal_point.y
            self.goal_point_marker.pose.position.z = 0.0
            self.pub_marker.publish(self.goal_point_marker)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
