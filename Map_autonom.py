#!/usr/bin/env python3
"""
Intelligenter Auto-Explorer für ROSMASTER X3
Erkennt Türen, merkt sich besuchte Bereiche, sucht neue Räume, stoppt wenn fertig
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math
import time
from collections import deque


class IntelligenterExplorer(Node):
    def __init__(self):
        super().__init__('intelligenter_explorer')
        
        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan0', self.lidar_cb, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.timer = self.create_timer(0.1, self.exploration_loop)
        
        # Status
        self.zustand = 'INIT'  # INIT, SCANNING, ZU_TUER, DURCH_TUER, EXPLORE, FERTIG
        self.position = None
        self.richtung = 0.0  # In Rad
        
        # Karte & Memory
        self.karte = None
        self.besuchte_punkte = []  # Wo war ich schon?
        self.tueren = []  # Gefundene Türen
        self.aktuelle_tuer = None
        self.letzter_neuer_bereich = time.time()
        
        # Lidar-Daten
        self.scan_data = None
        self.front_frei = True
        self.tuer_erkannt = False
        
        # Parameter
        self.min_tuer_breite = 0.8  # Meter (Tür muss mindestens 80cm breit sein)
        self.sicherheitsabstand = 0.4  # 40cm zu Wänden
        self.exploration_timeout = 60  # Sekunden ohne neuen Bereich = Fertig
        
        self.get_logger().info('Intelligenter Explorer gestartet!')
        self.get_logger().info('Warte auf Karte und Position...')

    def odom_cb(self, msg):
        """Aktuelle Position vom Odometrie-Topic"""
        self.position = msg.pose.pose
        # Quaternion zu Euler (Yaw)
        q = self.position.orientation
        self.richtung = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def map_cb(self, msg):
        """SLAM-Karte empfangen"""
        self.karte = msg
        
    def lidar_cb(self, scan):
        """Lidar auswerten: Hindernisse und Türen"""
        self.scan_data = scan
        
        # Front-Bereich prüfen (+-15 Grad)
        mitte = len(scan.ranges) // 2
        front = scan.ranges[mitte-10:mitte+10]
        gueltig = [r for r in front if 0.1 < r < 10.0]
        
        if gueltig:
            min_dist = min(gueltig)
            self.front_frei = min_dist > self.sicherheitsabstand
            
        # Türen erkennen (breite Lücken in den Seiten)
        self.tuer_erkannt = self.suche_tuer(scan)

    def suche_tuer(self, scan):
        """Sucht nach Türen (breite, freie Bereiche in den Seiten)"""
        if not scan:
            return False
            
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        # Linke und rechte Seite prüfen (90 Grad +- 30)
        links_mask = (angles > math.radians(60)) & (angles < math.radians(120))
        rechts_mask = (angles > math.radians(-120)) & (angles < math.radians(-60))
        
        tueren = []
        
        for seite, mask in [('links', links_mask), ('rechts', rechts_mask)]:
            seite_ranges = ranges[mask]
            seite_ranges = seite_ranges[np.isfinite(seite_ranges)]
            
            if len(seite_ranges) > 0:
                durchschnitt = np.mean(seite_ranges)
                if durchschnitt > self.min_tuer_breite:
                    # Potenzielle Tür gefunden
                    tueren.append(seite)
                    
        if tueren and self.zustand == 'EXPLORE':
            self.get_logger().info(f'Mögliche TÜR erkannt: {tueren[0]}')
            self.aktuelle_tuer = tueren[0]
            return True
            
        return False

    def ist_neuer_bereich(self):
        """Prüft ob wir an einem neuen Ort sind"""
        if not self.position or not self.besuchte_punkte:
            return True
            
        aktuell = (self.position.position.x, self.position.position.y)
        
        # Prüfe ob wir schon nah an einem besuchten Punkt waren (50cm Toleranz)
        for punkt in self.besuchte_punkte[-50:]:  # Nur letzte 50 prüfen (Performance)
            dist = math.sqrt((aktuell[0] - punkt[0])**2 + (aktuell[1] - punkt[1])**2)
            if dist < 0.5:
                return False
                
        return True

    def exploration_loop(self):
        """Haupt-Exploration Loop"""
        if not self.position:
            return
            
        # Position speichern (alle 2 Sekunden)
        if int(time.time()) % 2 == 0:
            aktuell = (self.position.position.x, self.position.position.y)
            if self.ist_neuer_bereich():
                self.besuchte_punkte.append(aktuell)
                self.letzter_neuer_bereich = time.time()
                
        # Check: Sind wir fertig? (Lange kein neuer Bereich)
        zeit_ohne_neues = time.time() - self.letzter_neuer_bereich
        if zeit_ohne_neues > self.exploration_timeout and self.zustand != 'FERTIG':
            self.zustand = 'FERTIG'
            self.get_logger().info('✓ EXPLORATION FERTIG! Keine neuen Bereiche gefunden.')
            self.get_logger().info(f'  Besuchte Punkte: {len(self.besuchte_punkte)}')
            return

        cmd = Twist()
        
        # Zustandsmaschine
        if self.zustand == 'INIT':
            self.get_logger().info('Starte Exploration...')
            self.zustand = 'EXPLORE'
            
        elif self.zustand == 'EXPLORE':
            # Normale Exploration mit Hindernisvermeidung
            if self.tuer_erkannt and len(self.tueren) < 5:  # Max 5 Türen
                self.zustand = 'ZU_TUER'
                self.tueren.append(self.aktuelle_tuer)
                
            elif self.front_frei:
                # Vorwärts, leichte Zickzack-Bewegung zum besseren Scannen
                cmd.linear.x = 0.2
                cmd.angular.z = 0.1 * math.sin(time.time())  # Leichte Oszillation
            else:
                # Hindernis - links oder rechts drehen (wechselseitig)
                dreh_richtung = 1 if len(self.besuchte_punkte) % 2 == 0 else -1
                cmd.angular.z = 0.5 * dreh_richtung
                cmd.linear.x = 0.0
                
        elif self.zustand == 'ZU_TUER':
            # Zur Tür drehen und hindurchfahren
            if self.aktuelle_tuer == 'links':
                cmd.angular.z = 0.3  # Nach links drehen
                cmd.linear.x = 0.1
            else:
                cmd.angular.z = -0.3  # Nach rechts drehen
                cmd.linear.x = 0.1
                
            # Wenn Front frei und Tür-Seite nah -> durchfahren
            if self.front_frei:
                self.zustand = 'DURCH_TUER'
                self.get_logger().info('Fahre durch TÜR...')
                
        elif self.zustand == 'DURCH_TUER':
            # Gerade durch die Tür
            if self.front_frei:
                cmd.linear.x = 0.25  # Etwas schneller
                cmd.angular.z = 0.0
            else:
                # Drin im neuen Raum
                self.zustand = 'EXPLORE'
                self.get_logger().info('Neuer Raum erreicht! Weiter exploration...')
                
        elif self.zustand == 'FERTIG':
            # Stop und Ende
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return  # Nichts mehr senden
            
        self.cmd_pub.publish(cmd)
        
        # Status alle 5 Sekunden ausgeben
        if int(time.time()) % 5 == 0:
            self.get_logger().info(f'Status: {self.zustand} | Besucht: {len(self.besuchte_punkte)} | Türen: {len(self.tueren)}')

    def stop(self):
        """Notfall-Stop"""
        cmd = Twist()
        for _ in range(20):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)


def main():
    rclpy.init()
    explorer = IntelligenterExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.get_logger().info('Strg+C - Beende...')
    finally:
        explorer.stop()
        explorer.destroy_node()
        rclpy.shutdown()
        print('Exploration beendet.')


if __name__ == '__main__':
    main()
