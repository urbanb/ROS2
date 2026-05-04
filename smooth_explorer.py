#!/usr/bin/env python3
"""
Explorer X3 - Kontinuierliche Bewegung
Fährt smooth ohne Zucken
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math


class SmoothExplorer(Node):
    def __init__(self):
        super().__init__('smooth_explorer')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan0', self.lidar_cb, 10)
        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)
        
        # WICHTIG: Schnellerer Timer für smoothere Bewegung
        self.timer = self.create_timer(0.05, self.steuerung)  # 20Hz statt 10Hz
        
        self.front_frei = True
        self.drehen_seit = 0
        self.zustand = 'VORWAERTS'  # VORWAERTS oder DREHEN
        
        self.get_logger().info('SMOOTH Explorer gestartet!')
        self.get_logger().info('Der Roboter fährt in 2 Sekunden los...')
        
        # Kurze Pause damit Lidar Daten sammelt
        import time
        time.sleep(2)
        
    def odom_cb(self, msg):
        pass  # Nur für Debug, nicht nötig für Fahren
        
    def lidar_cb(self, scan):
        """Prüft Front-Bereich"""
        if not scan.ranges:
            return
            
        mitte = len(scan.ranges) // 2
        # Breiterer Front-Bereich für bessere Erkennung
        front = scan.ranges[mitte-20:mitte+20]
        gueltig = [r for r in front if 0.1 < r < 10.0]
        
        if gueltig:
            min_dist = min(gueltig)
            self.front_frei = min_dist > 0.6  # 60cm Sicherheitsabstand
            # Debug-Ausgabe nur alle 2 Sekunden
            if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:
                self.get_logger().info(f'Distanz vorne: {min_dist:.2f}m | Frei: {self.front_frei}')
        
    def steuerung(self):
        """Kontinuierliche Steuerung"""
        cmd = Twist()
        
        if self.zustand == 'VORWAERTS':
            if self.front_frei:
                # Höhere Geschwindigkeit für smootheres Fahren
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
            else:
                # Hindernis! Starte Drehung
                self.zustand = 'DREHEN'
                self.drehen_seit = 0
                self.get_logger().info('Hindernis! Drehe...')
                
        elif self.zustand == 'DREHEN':
            # Drehe für 2 Sekunden (40 * 0.05s)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Links drehen
            
            self.drehen_seit += 1
            if self.drehen_seit > 40:  # 2 Sekunden gedreht
                self.zustand = 'VORWAERTS'
                self.get_logger().info('Weiter vorwärts...')
        
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SmoothExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop mit mehreren Nachrichten
        for _ in range(20):
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
