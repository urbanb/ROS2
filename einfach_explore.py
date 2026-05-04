#!/usr/bin/env python3
"""
Einfacher Auto-Explorer - fährt sofort los
Keine Odometrie nötig, nur Lidar
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random


class EinfacherExplorer(Node):
    def __init__(self):
        super().__init__('einfacher_explorer')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan0', self.lidar_cb, 10)
        self.timer = self.create_timer(0.1, self.fahren)
        
        self.front_frei = True
        self.links_frei = True
        self.rechts_frei = True
        
        self.get_logger().info('EINFACHER Explorer startet in 3 Sekunden...')
        self.get_logger().info('ACHTUNG: Roboter fährt los!')
        
    def lidar_cb(self, scan):
        """Prüft Front, Links, Rechts"""
        if not scan.ranges:
            return
            
        mitte = len(scan.ranges) // 2
        
        # Front (Mitte ±10)
        front = scan.ranges[mitte-10:mitte+10]
        front = [r for r in front if 0.1 < r < 10.0]
        self.front_frei = min(front) > 0.5 if front else False
        
        # Links (90° ±15)
        links_idx = int(mitte * 0.5)  # ca. 90°
        links = scan.ranges[links_idx-10:links_idx+10]
        links = [r for r in links if 0.1 < r < 10.0]
        self.links_frei = min(links) > 0.6 if links else False
        
        # Rechts (270° ±15)  
        rechts_idx = int(mitte * 1.5)  # ca. 270°
        rechts = scan.ranges[rechts_idx-10:rechts_idx+10]
        rechts = [r for r in rechts if 0.1 < r < 10.0]
        self.rechts_frei = min(rechts) > 0.6 if rechts else False
        
    def fahren(self):
        """Einfache Hindernisvermeidung"""
        cmd = Twist()
        
        if self.front_frei:
            # Geradeaus fahren
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
        else:
            # Hindernis! Wo ist mehr Platz?
            cmd.linear.x = 0.0
            
            if self.links_frei and not self.rechts_frei:
                cmd.angular.z = 0.4  # Nach links
            elif self.rechts_frei and not self.links_frei:
                cmd.angular.z = -0.4  # Nach rechts
            else:
                # Beide Seiten frei oder beide blockiert -> zufällig
                cmd.angular.z = 0.4 if random.random() > 0.5 else -0.4
                
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = EinfacherExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop
        for _ in range(10):
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
