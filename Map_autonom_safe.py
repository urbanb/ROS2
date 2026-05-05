#!/usr/bin/env python3
"""
Map_autonom_safe.py - Autonome Navigation mit Kollisionserkennung
ROS2 Humble | Python3

Features:
- Verwendet /odom_raw (korrekte Odometrie)
- Aktive Kollisionserkennung via LiDAR
- Notfall-Stop und Ausweichmanöver
- Tür-Erkennung und Gedächtnis
- Frontiers-Exploration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import numpy as np
import math
import random
import json
import os
from datetime import datetime
from collections import deque

# =============================================================================
# KONFIGURATION
# =============================================================================
SAFETY_CONFIG = {
    'min_front_distance': 0.35,      # Mindestabstand vorne (m)
    'critical_distance': 0.20,       # Kritischer Abstand → Notfall-Stop (m)
    'side_clearance': 0.25,          # Seitlicher Mindestabstand (m)
    'front_check_angle': math.pi/6,  # ±30° Vorne prüfen
    'recovery_turn_speed': 0.5,      # Drehgeschwindigkeit bei Hindernis
    'recovery_duration': 2.0,        # Sekunden zum Ausweichen
}

MAP_CONFIG = {
    'exploration_speed': 0.15,
    'turn_speed': 0.3,
    'min_obstacle_dist': 0.4,
    'frontier_threshold': 15,
    'visited_radius': 0.3,
    'door_threshold': 0.8,
}

MEMORY_FILE = os.path.expanduser('~/robot_memory.json')


# =============================================================================
# KOLLISIONSERKENNUNG
# =============================================================================
class CollisionDetector:
    """LiDAR-basierte Kollisionserkennung mit Ausweichlogik"""
    
    def __init__(self, config):
        self.config = config
        self.last_scan = None
        self.emergency_stop = False
        self.best_escape_direction = 0  # 0=gerade, >0=rechts, <0=links
        
    def update_scan(self, scan_msg):
        """Neuer LiDAR-Scan empfangen"""
        self.last_scan = scan_msg
        self._analyze_obstacles()
        
    def _analyze_obstacles(self):
        """Hindernisse analysieren und Escape-Richtung berechnen"""
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
            
            # KRITISCH → Sofort-Stop
            if min_front < self.config['critical_distance']:
                self.emergency_stop = True
            # WARNUNG → Langsam fahren / ausweichen
            elif min_front < self.config['min_front_distance']:
                self.emergency_stop = True
            else:
                self.emergency_stop = False
        else:
            self.emergency_stop = False
            
        # BESTE AUSWEICHRICHTUNG finden
        self._find_escape_direction(valid_ranges, valid_angles)
        
    def _find_escape_direction(self, ranges, angles):
        """Freieste Richtung für Ausweichmanöver finden"""
        # Linke und rechte Seite analysieren
        left_mask = (angles > math.pi/6) & (angles < math.pi * 2/3)
        right_mask = (angles < -math.pi/6) & (angles > -math.pi * 2/3)
        
        left_clearance = np.mean(ranges[left_mask]) if np.any(left_mask) else 0
        right_clearance = np.mean(ranges[right_mask]) if np.any(right_mask) else 0
        
        # Preferierte Richtung (positive = rechts, negative = links)
        if right_clearance > left_clearance + 0.1:
            self.best_escape_direction = 1.0  # Rechts ist freier
        elif left_clearance > right_clearance + 0.1:
            self.best_escape_direction = -1.0  # Links ist freier
        else:
            self.best_escape_direction = random.choice([-1.0, 1.0])  # Zufällig
            
    def get_safety_velocity(self, cmd_vel):
        """Befehl durch Sicherheitsfilter laufen lassen"""
        if not self.emergency_stop:
            return cmd_vel
            
        # Notfall-Stop oder Ausweichen
        safe_cmd = Twist()
        
        if self.last_scan is None:
            # Kein Scan → Stehen bleiben
            return safe_cmd
            
        # Ausweichmanöver: Drehen in die freie Richtung
        safe_cmd.angular.z = self.config['recovery_turn_speed'] * self.best_escape_direction
        safe_cmd.linear.x = -0.05  # Leicht rückwärts
        
        return safe_cmd
        
    def is_path_clear(self, direction='front', distance=None):
        """Prüfen ob Pfad in Richtung frei ist"""
        if self.last_scan is None:
            return False
            
        distance = distance or self.config['min_front_distance']
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
class AutonomousMapper(Node):
    def __init__(self):
        super().__init__('autonomous_mapper_safe')
        
        # QoS für Sensor-Daten
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/explorer/status', 10)
        
        # Kollisionsdetektor
        self.collision_detector = CollisionDetector(SAFETY_CONFIG)
        
        # State
        self.position = None
        self.orientation = 0.0
        self.map_data = None
        self.map_info = None
        self.is_exploring = False
        self.visited_positions = deque(maxlen=1000)
        self.known_doors = []
        self.state = 'IDLE'  # IDLE, EXPLORING, AVOIDING, RETURNING
        
        # Timer
        self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.load_memory()
        self.get_logger().info('🤖 AutonomousMapper mit Kollisionserkennung gestartet!')
        
    def odom_callback(self, msg):
        """/odom_raw Verarbeitung"""
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # Quaternion → Yaw
        q = msg.pose.pose.orientation
        self.orientation = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        if self.position not in self.visited_positions:
            self.visited_positions.append(self.position)
            
    def scan_callback(self, msg):
        """LiDAR-Scan für Kollisionserkennung"""
        self.collision_detector.update_scan(msg)
        
    def map_callback(self, msg):
        """Karten-Update"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
    def load_memory(self):
        """Gedächtnis laden"""
        if os.path.exists(MEMORY_FILE):
            try:
                with open(MEMORY_FILE, 'r') as f:
                    data = json.load(f)
                    self.known_doors = data.get('doors', [])
                    self.get_logger().info(f'💾 {len(self.known_doors)} Türen geladen')
            except Exception as e:
                self.get_logger().warn(f'Gedächtnis laden fehlgeschlagen: {e}')
                
    def save_memory(self):
        """Gedächtnis speichern"""
        try:
            data = {
                'doors': self.known_doors,
                'last_run': datetime.now().isoformat(),
                'visited_count': len(self.visited_positions)
            }
            with open(MEMORY_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Gedächtnis speichern fehlgeschlagen: {e}')
            
    def world_to_map(self, x, y):
        """Welt- zu Kartenkoordinaten"""
        if self.map_info is None:
            return None
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (mx, my) if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height else None
        
    def find_frontier(self):
        """Nächste unerforschte Grenze finden"""
        if self.map_data is None or self.position is None:
            return None
            
        current_cell = self.world_to_map(self.position[0], self.position[1])
        if current_cell is None:
            return None
            
        # BFS nach Frontier-Zellen
        h, w = self.map_data.shape
        visited = set([current_cell])
        queue = deque([(current_cell, 0)])
        frontiers = []
        
        while queue and len(frontiers) < 10:
            (cx, cy), dist = queue.popleft()
            
            # Prüfen ob Frontier (bekanntes Gebiet neben unbekanntem)
            if self.map_data[cy, cx] >= 0:
                neighbors = [(cx+1,cy), (cx-1,cy), (cx,cy+1), (cx,cy-1)]
                for nx, ny in neighbors:
                    if 0 <= nx < w and 0 <= ny < h and self.map_data[ny, nx] == -1:
                        frontiers.append(((cx, cy), dist))
                        break
                        
            # Weiter suchen
            for nx, ny in [(cx+1,cy), (cx-1,cy), (cx,cy+1), (cx,cy-1)]:
                if (nx, ny) not in visited and 0 <= nx < w and 0 <= ny < h:
                    if self.map_data[ny, nx] >= 0:
                        visited.add((nx, ny))
                        queue.append(((nx, ny), dist + 1))
                        
        if not frontiers:
            return None
            
        # Nächste Frontier wählen
        frontiers.sort(key=lambda x: x[1])
        fx, fy = frontiers[0][0]
        
        # Zurück zu Weltkoordinaten
        wx = fx * self.map_info.resolution + self.map_info.origin.position.x
        wy = fy * self.map_info.resolution + self.map_info.origin.position.y
        return (wx, wy)
        
    def detect_door(self):
        """Tür-Erkennung via LiDAR-Scan"""
        if self.collision_detector.last_scan is None:
            return None
            
        scan = self.collision_detector.last_scan
        ranges = np.array(scan.ranges)
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment/2,
                          scan.angle_increment)
        
        # Lücken in der vorderen Wand suchen
        front_mask = np.abs(angles[:len(ranges)]) < math.pi/4
        front_ranges = ranges[front_mask]
        
        if len(front_ranges) < 10:
            return None
            
        # Gruppen von Nah- und Fernwerten finden
        nearby = front_ranges < MAP_CONFIG['door_threshold']
        
        # Tür = Lücke zwischen nahen Werten
        in_doorway = False
        door_center = None
        
        for i, is_near in enumerate(nearby):
            if not is_near and not in_doorway:
                # Potenzielle Tür entdeckt
                door_start = i
                in_doorway = True
            elif is_near and in_doorway:
                # Tür endet
                door_end = i
                door_width = door_end - door_start
                if 3 <= door_width <= 20:  # Plausible Türgröße
                    door_center_idx = door_start + door_width // 2
                    door_angles = angles[:len(ranges)][front_mask]
                    door_center = door_angles[door_center_idx]
                    break
                in_doorway = False
                
        return door_center
        
    def control_loop(self):
        """Haupt-Steuerungs-Loop"""
        if self.position is None:
            return
            
        cmd = Twist()
        
        # === KOLLISIONSERKENNUNG ===
        if self.collision_detector.emergency_stop:
            # Notfall-Ausweichmanöver
            safe_cmd = self.collision_detector.get_safety_velocity(cmd)
            self.cmd_vel_pub.publish(safe_cmd)
            self.state = 'AVOIDING'
            return
            
        # === NORMALE NAVIGATION ===
        frontier = self.find_frontier()
        
        if frontier is None:
            self.get_logger().info('✅ Exploration abgeschlossen!')
            self.save_memory()
            return
            
        # Ziel-Richtung berechnen
        dx = frontier[0] - self.position[0]
        dy = frontier[1] - self.position[1]
        target_angle = math.atan2(dy, dx)
        
        # Tür-Erkennung
        door_angle = self.detect_door()
        if door_angle is not None:
            # Durch Tür navigieren
            angle_diff = door_angle - self.orientation
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            if abs(angle_diff) > 0.2:
                cmd.angular.z = MAP_CONFIG['turn_speed'] * (1 if angle_diff > 0 else -1)
            else:
                cmd.linear.x = MAP_CONFIG['exploration_speed']
        else:
            # Normale Frontier-Navigation
            angle_diff = target_angle - self.orientation
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            if abs(angle_diff) > 0.3:
                cmd.angular.z = MAP_CONFIG['turn_speed'] * (1 if angle_diff > 0 else -1)
            else:
                # Nur vorwärts wenn Weg frei
                if self.collision_detector.is_path_clear('front', MAP_CONFIG['min_obstacle_dist']):
                    cmd.linear.x = MAP_CONFIG['exploration_speed']
                else:
                    # Seitlich ausweichen
                    if self.collision_detector.is_path_clear('left'):
                        cmd.angular.z = MAP_CONFIG['turn_speed']
                    elif self.collision_detector.is_path_clear('right'):
                        cmd.angular.z = -MAP_CONFIG['turn_speed']
                    else:
                        cmd.angular.z = MAP_CONFIG['turn_speed']  # Drehen bis frei
                        
        # Sicherheits-Filter (zusätzliche Absicherung)
        safe_cmd = self.collision_detector.get_safety_velocity(cmd)
        self.cmd_vel_pub.publish(safe_cmd)


# =============================================================================
# MAIN
# =============================================================================
def main():
    rclpy.init()
    node = AutonomousMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Beendet durch Benutzer')
    finally:
        node.save_memory()
        # Stop-Befehl senden
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
