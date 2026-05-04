#!/usr/bin/env python3
"""
Robuster Explorer für ROSMASTER X3
Einfach, stabil, fährt sofort
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class RobustExplorer(Node):
    def __init__(self):
        super().__init__('robust_explorer')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan0', self.lidar_cb, 10)
        
        # Timer: 10Hz ist ausreichend
        self.timer = self.create_timer(0.1, self.fahren)
        
        self.front_frei = True
        self.seite_frei = 'links'  # 'links' oder 'rechts'
        self.dreh_zaehler = 0
        
        self.get_logger().info('='*50)
        self.get_logger().info('ROBUSTER EXPLORER')
        self.get_logger().info('Warte auf Lidar-Daten...')
        self.get_logger().info('Der Roboter fährt automatisch los!')
        self.get_logger().info('Strg+C zum Stoppen')
        self.get_logger().info('='*50)
        
    def lidar_cb(self, scan):
        """Einfache Hinderniserkennung"""
        if not scan.ranges or len(scan.ranges) < 100:
            return
            
        n = len(scan.ranges)
        mitte = n // 2
        
        # FRONT: -15° bis +15°
        front_ranges = scan.ranges[mitte-10:mitte+10]
        front_valid = [r for r in front_ranges if 0.05 < r < 30.0]
        front_min = min(front_valid) if front_valid else 10.0
        
        # LINKS: 60° bis 120°
        links_idx = int(n * 0.25)
        links_ranges = scan.ranges[links_idx-15:links_idx+15]
        links_valid = [r for r in links_ranges if 0.05 < r < 30.0]
        links_avg = sum(links_valid) / len(links_valid) if links_valid else 0
        
        # RECHTS: 240° bis 300°
        rechts_idx = int(n * 0.75)
        rechts_ranges = scan.ranges[rechts_idx-15:rechts_idx+15]
        rechts_valid = [r for r in rechts_ranges if 0.05 < r < 30.0]
        rechts_avg = sum(rechts_valid) / len(rechts_valid) if rechts_valid else 0
        
        # Entscheidung
        self.front_frei = front_min > 0.5
        self.seite_frei = 'links' if links_avg > rechts_avg else 'rechts'
        
        # Debug alle 3 Sekunden
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now % 3 == 0:
            status = "FREI" if self.front_frei else "BLOCKIERT"
            self.get_logger().info(f'Vorne: {front_min:.2f}m ({status}) | Seite: {self.seite_frei}')
        
    def fahren(self):
        """Einfache Fahrlogik"""
        cmd = Twist()
        
        if self.front_frei and self.dreh_zaehler == 0:
            # Normal vorwärts fahren
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
        else:
            # Hindernis - drehen
            cmd.linear.x = 0.0
            
            if self.dreh_zaehler == 0:
                # Starte neue Drehung
                self.dreh_zaehler = 30  # 3 Sekunden bei 10Hz
                richtung = 1.0 if self.seite_frei == 'links' else -1.0
                cmd.angular.z = 0.6 * richtung
                self.get_logger().info(f'Drehe nach {self.seite_frei}...')
            else:
                # Weiterdrehen
                richtung = 1.0 if self.seite_frei == 'links' else -1.0
                cmd.angular.z = 0.6 * richtung
                self.dreh_zaehler -= 1
                
                if self.dreh_zaehler == 0:
                    self.get_logger().info('Drehung beendet, fahre weiter...')
        
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = RobustExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stoppe...')
    finally:
        # Sicherer Stop
        try:
            stop_msg = Twist()
            for _ in range(5):
                node.cmd_pub.publish(stop_msg)
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()
        print('Beendet.')


if __name__ == '__main__':
    main()
