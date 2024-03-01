import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

import numpy as np
from os.path import expanduser
from numpy import linalg as LA
from time import gmtime, strftime

home = expanduser('~')
file = open(strftime(home+'/test/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

class WaypointLogger(Node):

    def __init__(self):
        super().__init__('waypoint_logger')
        self.odom_sub = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.odom_sub
        
    def odom_callback(self, odom_data):
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        print(x, y)
        file.write('%f, %f' % (
                                odom_data.pose.pose.position.x,
                                odom_data.pose.pose.position.y,
                                ))

def main(args=None):
    rclpy.init(args=args)

    waypoint_logger = WaypointLogger()
    try:
        rclpy.spin(waypoint_logger)
    except KeyboardInterrupt:
        file.close()
        print("file closed")
        waypoint_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
