#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, PointStamped
from pyquaternion import Quaternion

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import csv
import math

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Load waypoints from csv file
        self.waypoints = np.zeros((0, 2))
        with open("waypoint_data.csv", newline="") as f_in:
            reader = csv.reader(f_in)
            for row in reader:
                self.waypoints = np.vstack([self.waypoints, [float(row[0]), float(row[1])]])

        # TODO: create ROS subscribers and publishers

        self.sub_odom = self.create_subscription(Odometry, 'ego_racecar/odom', self.pose_callback, 10)
        self.sub_transform = self.create_subscription(TransformStamped, 'tf', self.transform_callback, 10)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.pub_marker = self.create_publisher(Marker, 'goal_point', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lookahead_distance = 2.0
        self.prev_waypoint_distance = float('inf')
        self.goal_point = [0.0, 0.0]

        scale = 1.0
        self.goal_point_marker = Marker()
        self.goal_point_marker.header.frame_id =  "ego_racecar/base_link" # "/map"
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
    
    def transform_callback(self, transform_data):
        self.transform_data = transform_data
    

    def pose_callback(self, odom_data):

        # TODO: find the current waypoint to track using methods mentioned in lecture
        # Maybe find the closest waypoint then only look ahead from there
        for i in range(len(self.waypoints)):

            waypoint = self.waypoints[i]
            current_waypoint_distance = np.sqrt((waypoint[0] - odom_data.pose.pose.position.x)**2 + (waypoint[1] - odom_data.pose.pose.position.y)**2)
            
            if current_waypoint_distance < self.prev_waypoint_distance:
                self.closest_waypoint_index = i

                self.prev_waypoint_distance = current_waypoint_distance

        start = self.closest_waypoint_index
        end = len(self.waypoints)


        # Convert quaternion to Euler angles
        quaternion = Quaternion(odom_data.pose.pose.orientation.w, 
                        odom_data.pose.pose.orientation.x, 
                        odom_data.pose.pose.orientation.y, 
                        odom_data.pose.pose.orientation.z)

        # Get the yaw angle (heading)
        car_heading_angle = quaternion.yaw_pitch_roll[0]

        # Create a unit vector for the car's heading
        car_heading = np.array([math.cos(car_heading_angle), math.sin(car_heading_angle)])
        
        for waypoint_index in range(start, end):
            waypoint = self.waypoints[waypoint_index]
            waypoint_distance = np.sqrt((waypoint[0] - odom_data.pose.pose.position.x)**2 + (waypoint[1] - odom_data.pose.pose.position.y)**2)

            # Calculate the vector from the car to the waypoint
            car_to_waypoint = np.array([waypoint[0] - odom_data.pose.pose.position.x, waypoint[1] - odom_data.pose.pose.position.y])

            # Normalize the vector
            car_to_waypoint = car_to_waypoint / np.linalg.norm(car_to_waypoint)

            # Calculate the angle between the car's heading and the vector to the waypoint
            angle = math.acos(np.dot(car_heading, car_to_waypoint))

            # If the angle is less than 90 degrees, the waypoint is in front of the car
            if angle < math.pi / 2:
                if (waypoint_distance == self.lookahead_distance) and (waypoint_index > prev_index):
                    self.goal_point = waypoint
                    break

                elif (self.lookahead_distance - 0.5 < waypoint_distance < self.lookahead_distance + 0.5) and (waypoint_index > prev_index):
                    self.goal_point = waypoint
                    break

            prev_index = waypoint_index

        # TODO: transform goal point to vehicle frame of reference [x, y]
        vehicle_frame = 'ego_racecar/base_link'
        map_frame = 'map'
        try:
            t = self.tf_buffer.lookup_transform(
                vehicle_frame,
                map_frame,
                rclpy.time.Time())
            # print(t)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {vehicle_frame} to {map_frame}: {ex}')
            return
        vehicle_quaternion = Quaternion(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z)
        rotated_goal_point = vehicle_quaternion.rotate([self.goal_point[0], self.goal_point[1], 0.0])
        goal_point_vehicle_frame = [rotated_goal_point[0] + t.transform.translation.x, rotated_goal_point[1] + t.transform.translation.y]
        # # TODO: calculate curvature/steering angle
        curvature = 2*goal_point_vehicle_frame[1]/self.lookahead_distance**2

        # # TODO: publish drive message, don't forget to limit the steering angle.
        drive_data = AckermannDriveStamped()
        steering_angle = 0.5 * curvature
        drive_data.drive.steering_angle = steering_angle 
        drive_data.drive.speed = 0.1 * (1 / 1.2)**(steering_angle- 15)

        self.pub_drive.publish(drive_data)
        print(goal_point_vehicle_frame)
        self.goal_point_marker.pose.position.x = goal_point_vehicle_frame[0] 
        self.goal_point_marker.pose.position.y = goal_point_vehicle_frame[1]  
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
