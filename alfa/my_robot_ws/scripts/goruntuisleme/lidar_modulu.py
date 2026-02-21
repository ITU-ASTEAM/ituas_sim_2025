#!/usr/bin/env python3
"""
LIDAR Modülü - Gazebo Transport üzerinden LIDAR verisi okuma
withmap.py ile entegrasyon için tasarlandı
"""

import numpy as np
import math
from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan


class LidarModulu:
    def __init__(self, topic_name="/scan"):
        self.node = Node()
        self.topic_name = topic_name
        
        # LIDAR değişkenleri
        self.ranges = np.array([])
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.lidar_ready = False
        
        # LIDAR offset (robotun merkezinden LIDAR'a)
        self.lidar_offset_x = 0.5
        
        # Subscribe
        if not self.node.subscribe(LaserScan, topic_name, self._lidar_callback):
            print(f"[LIDAR] HATA: {topic_name} subscribe edilemedi!")
        else:
            print(f"[LIDAR] {topic_name} subscribe edildi")
    
    def _lidar_callback(self, msg):
        """LIDAR mesajını işle"""
        self.ranges = np.array(list(msg.ranges))
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        print("DEBUG: LIDAR MESAJI GELDİ!") # Bunu ekle
        if len(self.ranges) > 1:
            self.angle_increment = (self.angle_max - self.angle_min) / (len(self.ranges) - 1)
        
        if not self.lidar_ready:
            print(f"[LIDAR] ✓ Veri alınıyor ({len(self.ranges)} nokta)")
        self.lidar_ready = True
    
    def is_ready(self):
        return self.lidar_ready
    
    def get_scan_points(self, robot_pose, max_range=20.0):
        """
        LIDAR noktalarını global koordinatlara dönüştür
        
        Args:
            robot_pose: (x, y, yaw) tuple
            max_range: Maksimum mesafe filtresi (metre)
        
        Returns:
            np.array: (N, 2) şeklinde global (x, y) koordinatları
            np.array: (N,) şeklinde mesafe değerleri
        """
        if not self.lidar_ready or len(self.ranges) == 0:
            return np.empty((0, 2)), np.array([])
        
        rx, ry, r_yaw = robot_pose
        
        # Açı dizisi
        angles = np.linspace(self.angle_min, self.angle_max, len(self.ranges))
        
        # Geçerli noktaları filtrele (inf değil ve max_range içinde)
        valid_mask = (~np.isinf(self.ranges)) & (self.ranges > 0.3) & (self.ranges < max_range)
        valid_ranges = self.ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return np.empty((0, 2)), np.array([])
        
        # Robot frame koordinatları
        # LIDAR'ın yönlendirmesine göre sin/cos değişebilir
        x_robot = valid_ranges * np.cos(valid_angles) + self.lidar_offset_x
        y_robot = valid_ranges * np.sin(valid_angles)
        
        # Global frame'e dönüştür
        cos_yaw = np.cos(r_yaw)
        sin_yaw = np.sin(r_yaw)
        
        x_global = rx + (x_robot * cos_yaw - y_robot * sin_yaw)
        y_global = ry + (x_robot * sin_yaw + y_robot * cos_yaw)
        
        points = np.column_stack([x_global, y_global])
        
        return points, valid_ranges
    
    def get_obstacles(self, robot_pose, close_threshold=5.0, max_range=20.0):
        """
        Yakın engelleri tespit et
        
        Args:
            robot_pose: (x, y, yaw) tuple
            close_threshold: Yakın sayılacak mesafe (metre)
            max_range: Maksimum mesafe
        
        Returns:
            np.array: Yakın engellerin global koordinatları
        """
        points, ranges = self.get_scan_points(robot_pose, max_range)
        
        if len(ranges) == 0:
            return np.empty((0, 2))
        
        close_mask = ranges < close_threshold
        return points[close_mask]
    
    def cluster_points(self, points, cluster_distance=1.0):
        """
        Yakın noktaları kümele (basit clustering)
        
        Args:
            points: (N, 2) şeklinde nokta dizisi
            cluster_distance: Kümeleme mesafesi
        
        Returns:
            list: Küme merkezleri [(x, y), ...]
        """
        if len(points) == 0:
            return []
        
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if used[i]:
                continue
            
            # Bu noktaya yakın tüm noktaları bul
            distances = np.linalg.norm(points - points[i], axis=1)
            cluster_mask = distances < cluster_distance
            
            # Küme merkezi
            cluster_points = points[cluster_mask & ~used]
            if len(cluster_points) > 0:
                center = np.mean(cluster_points, axis=0)
                size = len(cluster_points)
                clusters.append({
                    'center': center,
                    'size': size
                })
            
            used[cluster_mask] = True
        
        return clusters


if __name__ == "__main__":
    # Test
    import time
    
    print("LIDAR Modülü Test")
    lidar = LidarModulu("/scan")
    
    # Pozisyon modülünden pose al (test için sabit)
    test_pose = (0.0, 0.0, 0.0)
    
    try:
        while True:
            time.sleep(0.5)
            if lidar.is_ready():
                points, ranges = lidar.get_scan_points(test_pose)
                print(f"Nokta sayısı: {len(points)}")
                
                clusters = lidar.cluster_points(points)
                print(f"Küme sayısı: {len(clusters)}")
    except KeyboardInterrupt:
        print("\nÇıkış...")
