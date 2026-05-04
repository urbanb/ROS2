#!/usr/bin/env python3
"""
Einfacher kontinuierlicher Fahrer für X3
Wie 'ros2 topic pub' aber mit Hindernisvermeidung
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
import time


class SimpleDrive(Node):
    def __init__(self):
        super().__init__('simple_drive')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan0', self.lidar_cb, 10)
        
        self.front_clear = True
        self.running = True
        
        self.get_logger().info('Simple Drive gestartet!')
        self.get_logger().info('Fährt in 2 Sekunden los...')
        
    def lidar_cb(self, scan):
        """Prüft Front"""
        if not scan.ranges:
            return
            
        mid = len(scan.ranges) // 2
        front = scan.ranges[mid-15:mid+15]
        valid = [r for r in front if 0.1 < r < 10.0]
        
        if valid:
            self.front_clear = min(valid) > 0.5
            
    def drive_loop(self):
        """Haupt-Loop wie ros2 topic pub --rate 10"""
        time.sleep(2)  # Warte auf Lidar
        
        while self.running and rclpy.ok():
            msg = Twist()
            
            if self.front_clear:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.5  # Drehen
                
            self.cmd_pub.publish(msg)
            time.sleep(0.1)  # 10 Hz wie ros2 topic pub
            
    def stop(self):
        self.running = False
        time.sleep(0.2)
        for _ in range(10):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)


def main():
    rclpy.init()
    node = SimpleDrive()
    
    # Drive-Loop in separatem Thread
    drive_thread = threading.Thread(target=node.drive_loop)
    drive_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        drive_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
