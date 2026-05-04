#!/usr/bin/env python3
"""
Korrigierter Explorer für ROSMASTER X3
Nutzt /odom_raw statt /odom
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
import random


class ExplorerX3(Node):
    def __init__(self):
        super().__init__('explorer_x3')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan0', self.lidar_cb, 10)
        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)  # WICHTIG: /odom_raw !
        
        self.timer = self.create_timer(0.1, self.steuerung)
        
        self.front_frei = True
        self.position = None
        self.richtung = 0
        
        self.get_logger().info('Explorer X3 gestartet - warte auf Daten...')
        
    def odom_cb(self, msg):
        """Odometrie empfangen"""
        self.position = msg.pose.pose
        q = msg.pose.pose.orientation
        self.richtung = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
        
    def lidar_cb(self, scan):
        """Lidar auswerten"""
        if not scan.ranges:
            return
            
        mitte = len(scan.ranges) // 2
        front = [r for r in scan.ranges[mitte-15:mitte+15] if 0.1 < r < 10.0]
        self.front_frei = min(front) > 0.5 if front else False
        
    def steuerung(self):
        """Fahrlogik"""
        cmd = Twist()
        
        if self.front_frei:
            cmd.linear.x = 0.15
        else:
            cmd.angular.z = 0.5 if random.random() > 0.5 else -0.5
            
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ExplorerX3()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
