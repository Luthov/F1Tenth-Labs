import pandas as pd
import numpy as np

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker

class VisualizeWaypoints(Node):

    def __init__(self):
        super().__init__('visualize_waypoints')
        
        self.waypoint_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        
        waypoint_dataframe = pd.read_csv('waypoints.csv')
        
        # Extract the first 2 columns in the waypoint_dataframe
        waypoints = waypoint_dataframe[['x', 'y']]
        

def main(args=None):
    rclpy.init(args=args)

    relay = Relay()

    rclpy.spin(relay)

    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
