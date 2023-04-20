#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class Test1(Node):

    def __init__(self):
        super().__init__('test1')
        self.drive_publisher_ = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.lidar_subscriber_ = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan_data = LaserScan()
        self.max_gap_index = 0


    def lidar_callback(self, msg):
        self.scan_data = msg

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (r, p, self.yaw) = euler_from_quaternion(orientation_list)
        # self.find_gap()
        #print(self.yaw)

    def find_gap(self):
        gap_size = 5
        ranges = self.scan_data.ranges
        last_index = len(ranges)-1
        print("Last Index: %i" % last_index)

        max_gap = 0

        for i in range(0,last_index-3):
            temp_sum = 0
            for j in range(i,i+4):
                temp_sum = temp_sum + ranges[j]
            
            if temp_sum > max_gap:
                max_gap = temp_sum
                self.max_gap_index = i + 2
            else:
                continue
        
        print("Max Gap Index: %i" % self.max_gap_index)


def main(args=None):
    rclpy.init(args=args)

    test1 = Test1()
    test1.find_gap()
    rclpy.spin(test1)

    test1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
