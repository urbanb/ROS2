#!/usr/bin/env python3
"""
Map_autonom_safe_nomap.py - Autonome Navigation OHNE Kartierung
Nur LiDAR-basiert - für schnellen Test!

Features:
- Verwendet /odom_raw (korrekte Odometrie)
- Aktive Kollisionserkennung via LiDAR
- Random Walk Exploration (keine Map nötig)
- Notfall-Stop und Ausweichmanöver
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import random
import time

# =============================================================================
# KONFIGURATION
# =============================================================================
SAFETY_CONFIG = {
    'min_front_distance': 0.35,      # Mindestabstand vorne (m)
    'critical_distance': 0.20,       # Kritischer Abstand → Notfall-Stop (m)
    'side_clearance': 0.25,          # Seitlicher Mindestabstand (m)
    'front_check_angle': math.pi/6,  # ±30° Vorne prüfen
    'recovery_turn_speed': 0.5,      # Drehgeschwindigkeit bei Hindernis
}

NAV_CONFIG = {
    'exploration_speed': 0.15,
    'turn_speed': 0.4,
    'drive_time': 3.0,               # Sekunden geradeaus fahren
    'turn_time': 1.0,                # Sekunden drehen
}


# =============================================================================
# KOLLISIONSERKENNUNG
# =============================================================================
class CollisionDetector:
    """LiDAR-basierte Kollisionserkennung"""
    
    def __init__(self, config):
        self.config = config
        self.last_scan = None
        self.emergency_stop = False
        self.best_escape_direction = 0
        
    def update_scan(self, scan_msg):
        """Neuer LiDAR-Scan empfangen"""
        self.last_scan = scan_msg
        self._analyze_obstacles()
        
    def _analyze_obstacles(self):
        """Hindernisse analysieren"""
        if self.last_scan is None:
            return
            
        scan = self.last_scan
        ranges = np.array(scan.ranges)
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment/2, 
                          scan.angle_increment)
        
        # Ungültige Werte filtern
        valid_mask = (ranges > scan.range_min) & (ranges < scan.range_max) & ~np.isnan(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[:len(valid_ranges)]
        
        if len(valid_ranges) == 0:
            self.emergency_stop = False
            return
            
        # FRONT-BEREICH analysieren (±30°)
        front_mask = np.abs(valid_angles) < self.config['front_check_angle']
        front_ranges = valid_ranges[front_mask]
        
        if len(front_ranges) > 0:
            min_front = np.min(front_ranges)
            
            if min_front < self.config['critical_distance']:
                self.emergency_stop = True
            elif min_front < self.config['min_front_distance']:
                self.emergency_stop = True
            else:
                self.emergency_stop = False
        else:
            self.emergency_stop = False
            
        # Beste Ausweichrichtung finden
        self._find_escape_direction(valid_ranges, valid_angles)
        
    def _find_escape_direction(self, ranges, angles):
        """Freieste Richtung finden"""
        left_mask = (angles > math.pi/6) & (angles < math.pi * 2/3)
        right_mask = (angles < -math.pi/6) & (angles > -math.pi * 2/3)
        
        left_clearance = np.mean(ranges[left_mask]) if np.any(left_mask) else 0
        right_clearance = np.mean(ranges[right_mask]) if np.any(right_mask) else 0
        
        if right_clearance > left_clearance + 0.1:
            self.best_escape_direction = 1.0
        elif left_clearance > right_clearance + 0.1:
            self.best_escape_direction = -1.0
        else:
            self.best_escape_direction = random.choice([-1.0, 1.0])
            
    def get_safety_velocity(self, cmd_vel):
        """Befehl durch Sicherheitsfilter"""
        if not self.emergency_stop:
            return cmd_vel
            
        safe_cmd = Twist()
        safe_cmd.angular.z = self.config['recovery_turn_speed'] * self.best_escape_direction
        safe_cmd.linear.x = -0.05  # Leicht rückwärts
        return safe_cmd
        
    def is_path_clear(self, direction='front', distance=0.4):
        """Prüfen ob Pfad frei ist"""
        if self.last_scan is None:
            return False
            
        scan = self.last_scan
        ranges = np.array(scan.ranges)
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment/2,
                          scan.angle_increment)
        
        valid_mask = (ranges > scan.range_min) & (ranges < scan.range_max)
        
        if direction == 'front':
            angle_mask = np.abs(angles[:len(ranges)]) < math.pi/8
        elif direction == 'left':
            angle_mask = (angles[:len(ranges)] > math.pi/4) & (angles[:len(ranges)] < math.pi*3/4)
        elif direction == 'right':
            angle_mask = (angles[:len(ranges)] < -math.pi/4) & (angles[:len(ranges)] > -math.pi*3/4)
        else:
            return False
            
        check_mask = valid_mask & angle_mask
        if not np.any(check_mask):
            return False
            
        return np.min(ranges[check_mask]) > distance


# =============================================================================
# HAUPTNODE
# =============================================================================
class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Kollisionsdetektor
        self.collision = CollisionDetector(SAFETY_CONFIG)
        
        # State
        self.position = None
        self.state = 'INIT'
        self.state_start_time = time.time()
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🤖 LiDAR-Explorer gestartet (keine Map nötig)!')
        
    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def scan_callback(self, msg):
        self.collision.update_scan(msg)
        
    def set_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'State: {self.state} → {new_state}')
            self.state = new_state
            self.state_start_time = time.time()
            
    def control_loop(self):
        cmd = Twist()
        
        # === KOLLISIONS-SCHUTZ ===
        if self.collision.emergency_stop:
            cmd.angular.z = SAFETY_CONFIG['recovery_turn_speed'] * self.collision.best_escape_direction
            cmd.linear.x = -0.05
            self.set_state('AVOIDING')
            self.cmd_vel_pub.publish(cmd)
            return
            
        # === STATE MACHINE ===
        elapsed = time.time() - self.state_start_time
        
        if self.state == 'INIT':
            # Kurz warten auf ersten Scan
            if self.collision.last_scan is not None:
                self.set_state('DRIVING')
            
        elif self.state == 'DRIVING':
            # Geradeaus fahren
            if self.collision.is_path_clear('front', NAV_CONFIG['min_front_distance']):
                cmd.linear.x = NAV_CONFIG['exploration_speed']
                if elapsed > NAV_CONFIG['drive_time']:
                    self.set_state('TURNING')
            else:
                # Hindernis → Drehen
                self.set_state('TURNING')
                
        elif self.state == 'TURNING':
            # In freie Richtung drehen
            if self.collision.is_path_clear('left', NAV_CONFIG['min_front_distance']):
                if not self.collision.is_path_clear('right', NAV_CONFIG['min_front_distance']):
                    cmd.angular.z = NAV_CONFIG['turn_speed']  # Links ist freier
                else:
                    cmd.angular.z = random.choice([-NAV_CONFIG['turn_speed'], NAV_CONFIG['turn_speed']])
            else:
                cmd.angular.z = -NAV_CONFIG['turn_speed']  # Rechts drehen
                
            if elapsed > NAV_CONFIG['turn_time'] and self.collision.is_path_clear('front', NAV_CONFIG['min_front_distance']):
                self.set_state('DRIVING')
                
        elif self.state == 'AVOIDING':
            # Nach Ausweichen wieder fahren
            if not self.collision.emergency_stop and self.collision.is_path_clear('front', NAV_CONFIG['min_front_distance']):
                self.set_state('DRIVING')
                
        # Sicherheits-Filter
        safe_cmd = self.collision.get_safety_velocity(cmd)
        self.cmd_vel_pub.publish(safe_cmd)


# =============================================================================
# MAIN
# =============================================================================
def main():
    rclpy.init()
    node = SimpleExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Beendet')
    finally:
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
