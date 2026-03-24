#!/usr/bin/env python3
"""
Mistik SLAM v6 - Optimized Version
Daha az tekrar, daha stabil mapping
"""

import cv2
import numpy as np
import math
import time
import os
from collections import defaultdict
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.msg import Costmap
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
from kamera_modulu import KameraModulu
from position import PozisyonModulu
from lidar_modulu import LidarModulu

class MistikHaritaSistemi:
    def __init__(self):
        print("\n" + "=" * 60)
        print("MİSTİK SLAM v6 - OPTİMİZE EDİLMİŞ")
        print("=" * 60 + "\n")
        
        # Harita ayarları
        self.merge_distance = 4.0      # 4m içindekiler AYNI nesne
        self.map_size = 1200
        self.scale = 8
        
        # Kamera
        self.camera_offset_x = 0.5
        
        # Modeller
        print("[1/4] YOLO yükleniyor...")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, "best29jan.pt")

        if not os.path.exists(weights_path):
            raise FileNotFoundError(f"YOLO ağırlık dosyası bulunamadı: {weights_path}")

        self.model = YOLO(weights_path)
        print("      ✓ Hazır")
        
        print("[2/4] Kameralar...")
        self.cam_rgb = KameraModulu(topic_name="/camera/image_raw")
        self.cam_depth = KameraModulu(topic_name="/camera/depth/image_raw")
        print("      ✓ Hazır")
        
        print("[3/5] Pozisyon...")
        self.pozisyon = PozisyonModulu()
        
        if not self.pozisyon.is_ready():
            print("      ✗ HATA!")
            exit(1)
        print("      ✓ Hazır")
        
        print("[4/5] LIDAR...")
        self.lidar = LidarModulu(topic_name="/lidar")
        self.lidar_points = np.empty((0, 2))  # Global LIDAR noktaları
        self.lidar_clusters = []  # LIDAR kümeleri
        self.lidar_max_range = 20.0  # Maksimum LIDAR mesafesi
        print("      ✓ Hazır")
        
        # Nesne listesi
        self.detected_objects = []
        
        # Kamera params
        self.focal_length = 554.25
        self.image_width = 640
        self.image_height = 480
        
        # Buffer - Daha sıkı filtre
        self.detection_buffer = defaultdict(list)
        self.buffer_size = 10
        self.min_confirmations = 6  # Daha yüksek
        
        self.colors = {
            'yellow_buoy': (0, 255, 255),
            'red_buoy': (0, 0, 255),
            'green_buoy': (0, 255, 0),
            'blue_buoy': (255, 0, 0),
            'black_buoy': (100, 100, 100),
            'orange_buoy': (0, 165, 255),
            # Alternatif isimler
            'yellow': (0, 255, 255),
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'black': (100, 100, 100),
            'orange': (0, 165, 255),
            'white': (255, 255, 255)
        }
        
        print("[5/5] Sistem hazır!")
        print("=" * 60)
        print("Tuşlar: q=Çıkış | r=Reset | c=Temizle | l=LIDAR toggle")
        print("=" * 60 + "\n")
        
        # LIDAR görselleştirme açık/kapalı
        self.show_lidar = True

        # RViz yayıncıları
        self.rviz_frame_id = "map"
        self.rviz_enabled = self._init_rviz_publishers()
        if self.rviz_enabled:
            print("[RViz] Görselleştirme aktif: /mistik_slam/markers, /mistik_slam/lidar_points")
            print("[RViz] OpenCV pencere açılmayacak; veriler RViz'e yayınlanıyor.\n")

    def _init_rviz_publishers(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.rviz_node = rclpy.create_node('mistik_slam_rviz_publisher')
            self.rviz_node.set_parameters([
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ])

            self.marker_pub = self.rviz_node.create_publisher(MarkerArray, '/mistik_slam/markers', 10)
            self.lidar_cloud_pub = self.rviz_node.create_publisher(PointCloud2, '/mistik_slam/lidar_points', 10)

            # Nav2 obstacle layer için tespit edilen dubalar
            # QoS: transient_local → Nav2 başlarken geçmiş mesajları da alır
            from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
            obs_qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
            self.obstacle_pub = self.rviz_node.create_publisher(
                PointCloud2,
                '/detected_buoys',   # Nav2 obstacle_layer bu topic'i dinleyecek
                obs_qos
            )
            self.last_obstacle_pub = 0.0  # throttle için
            return True
        except Exception as exc:
            print(f"[RViz] Başlatılamadı: {exc}")
            self.rviz_node = None
            return False

    def publish_obstacles(self):
        """
        Güvenilir dubaları Nav2 obstacle_layer'a PointCloud2 olarak yayınla.
        Sadece confidence > MIN_CONF olanlar gönderilir.
        Her 0.5 saniyede bir yayınlanır (throttle).
        """
        if not self.rviz_enabled or self.rviz_node is None or not rclpy.ok():
            return

        now = time.time()
        if now - self.last_obstacle_pub < 0.5:  # 2 Hz yeterli
            return
        self.last_obstacle_pub = now

        MIN_CONF = 15  # Bu değerin altındakiler Nav2'ye gönderilmez

        # Güvenilir dubaları filtrele
        reliable = [
            obj for obj in self.detected_objects
            if obj['confidence'] >= MIN_CONF
        ]

        if not reliable:
            # Boş cloud gönder — Nav2 temizlesin
            header = Header()
            header.stamp = self.rviz_node.get_clock().now().to_msg()
            header.frame_id = 'map'
            empty_cloud = pc2.create_cloud_xyz32(header, [])
            try:
                self.obstacle_pub.publish(empty_cloud)
            except Exception:
                pass
            return

        # XYZ noktaları — Z=0.5 (Nav2 3D costmap için biraz yükseklik ver)
        points = [
            [float(obj['x']), float(obj['y']), 0.5]
            for obj in reliable
        ]

        header = Header()
        header.stamp = self.rviz_node.get_clock().now().to_msg()
        header.frame_id = 'map'

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        try:
            self.obstacle_pub.publish(cloud_msg)
        except Exception as e:
            print(f'[obstacle_pub] Hata: {e}')
            return

        # Log — sadece değişince
        if not hasattr(self, '_last_obs_count') or self._last_obs_count != len(reliable):
            self._last_obs_count = len(reliable)
            print(f'[Nav2] {len(reliable)} duba obstacle olarak yayınlandı '
                  f'(conf>={MIN_CONF})')

    def _yaw_to_quaternion(self, yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def _bgr_to_rgba(self, bgr, alpha=1.0):
        return (
            float(bgr[2]) / 255.0,
            float(bgr[1]) / 255.0,
            float(bgr[0]) / 255.0,
            alpha,
        )

    def publish_rviz(self, robot_pose):
        if not self.rviz_enabled or self.rviz_node is None or not rclpy.ok():
            return

        stamp = self.rviz_node.get_clock().now().to_msg()
        rx, ry, r_yaw = robot_pose

        # 1) LIDAR PointCloud2
        header = Header()
        header.stamp = stamp
        header.frame_id = self.rviz_frame_id
        lidar_xyz = [[float(p[0]), float(p[1]), 0.0] for p in self.lidar_points] if len(self.lidar_points) > 0 else []
        lidar_msg = pc2.create_cloud_xyz32(header, lidar_xyz)
        try:
            self.lidar_cloud_pub.publish(lidar_msg)
        except Exception:
            return

        # 2) MarkerArray
        markers = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = self.rviz_frame_id
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        marker_id = 0

        # Robot gövdesi
        robot_marker = Marker()
        robot_marker.header.frame_id = self.rviz_frame_id
        robot_marker.header.stamp = stamp
        robot_marker.ns = 'robot'
        robot_marker.id = marker_id
        marker_id += 1
        robot_marker.type = Marker.CUBE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = float(rx)
        robot_marker.pose.position.y = float(ry)
        robot_marker.pose.position.z = 0.20
        qx, qy, qz, qw = self._yaw_to_quaternion(r_yaw)
        robot_marker.pose.orientation.x = qx
        robot_marker.pose.orientation.y = qy
        robot_marker.pose.orientation.z = qz
        robot_marker.pose.orientation.w = qw
        robot_marker.scale.x = 2.0
        robot_marker.scale.y = 1.0
        robot_marker.scale.z = 0.25
        robot_marker.color.r = 1.0
        robot_marker.color.g = 0.8
        robot_marker.color.b = 0.8
        robot_marker.color.a = 0.95
        markers.markers.append(robot_marker)

        # Robot yön oku
        heading_marker = Marker()
        heading_marker.header.frame_id = self.rviz_frame_id
        heading_marker.header.stamp = stamp
        heading_marker.ns = 'robot_heading'
        heading_marker.id = marker_id
        marker_id += 1
        heading_marker.type = Marker.ARROW
        heading_marker.action = Marker.ADD
        heading_marker.scale.x = 0.15
        heading_marker.scale.y = 0.30
        heading_marker.scale.z = 0.30
        heading_marker.color.r = 1.0
        heading_marker.color.g = 0.1
        heading_marker.color.b = 0.1
        heading_marker.color.a = 1.0
        p0 = Point()
        p0.x, p0.y, p0.z = float(rx), float(ry), 0.35
        p1 = Point()
        p1.x = float(rx + 3.5 * math.cos(r_yaw))
        p1.y = float(ry + 3.5 * math.sin(r_yaw))
        p1.z = 0.35
        heading_marker.points = [p0, p1]
        markers.markers.append(heading_marker)

        # LIDAR kümeleri
        if self.show_lidar:
            for cluster in self.lidar_clusters:
                center = cluster['center']
                cluster_marker = Marker()
                cluster_marker.header.frame_id = self.rviz_frame_id
                cluster_marker.header.stamp = stamp
                cluster_marker.ns = 'lidar_clusters'
                cluster_marker.id = marker_id
                marker_id += 1
                cluster_marker.type = Marker.SPHERE
                cluster_marker.action = Marker.ADD
                cluster_marker.pose.position.x = float(center[0])
                cluster_marker.pose.position.y = float(center[1])
                cluster_marker.pose.position.z = 0.10
                cluster_marker.pose.orientation.w = 1.0
                size = min(cluster['size'] * 0.04 + 0.12, 0.55)
                cluster_marker.scale.x = size
                cluster_marker.scale.y = size
                cluster_marker.scale.z = 0.12
                cluster_marker.color.r = 0.60
                cluster_marker.color.g = 0.90
                cluster_marker.color.b = 1.00
                cluster_marker.color.a = 0.80
                markers.markers.append(cluster_marker)

        # Vision nesneleri
        sorted_objects = sorted(self.detected_objects, key=lambda x: x['confidence'])
        for obj in sorted_objects:
            conf_ratio = min(float(obj['confidence']) / 50.0, 1.0)
            rr, gg, bb, aa = self._bgr_to_rgba(obj['color'], alpha=0.90)

            obj_marker = Marker()
            obj_marker.header.frame_id = self.rviz_frame_id
            obj_marker.header.stamp = stamp
            obj_marker.ns = 'vision_objects'
            obj_marker.id = marker_id
            marker_id += 1
            obj_marker.type = Marker.SPHERE
            obj_marker.action = Marker.ADD
            obj_marker.pose.position.x = float(obj['x'])
            obj_marker.pose.position.y = float(obj['y'])
            obj_marker.pose.position.z = 0.30
            obj_marker.pose.orientation.w = 1.0
            diameter = 0.30 + 0.35 * conf_ratio
            obj_marker.scale.x = diameter
            obj_marker.scale.y = diameter
            obj_marker.scale.z = 0.30
            obj_marker.color.r = rr
            obj_marker.color.g = gg
            obj_marker.color.b = bb
            obj_marker.color.a = aa
            markers.markers.append(obj_marker)

            if obj['confidence'] > 20:
                text_marker = Marker()
                text_marker.header.frame_id = self.rviz_frame_id
                text_marker.header.stamp = stamp
                text_marker.ns = 'vision_conf_text'
                text_marker.id = marker_id
                marker_id += 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = float(obj['x'])
                text_marker.pose.position.y = float(obj['y'])
                text_marker.pose.position.z = 0.85
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.28
                text_marker.color.r = rr
                text_marker.color.g = gg
                text_marker.color.b = bb
                text_marker.color.a = 1.0
                text_marker.text = f"{obj['type']} ({obj['confidence']:.1f})"
                markers.markers.append(text_marker)

        # İstatistik metni
        high_conf = sum(1 for o in self.detected_objects if o['confidence'] > 20)
        med_conf = sum(1 for o in self.detected_objects if 10 <= o['confidence'] <= 20)
        low_conf = sum(1 for o in self.detected_objects if o['confidence'] < 10)
        lidar_status = f"LIDAR: {len(self.lidar_points)} pkt, {len(self.lidar_clusters)} kume" if self.show_lidar else "LIDAR: KAPALI"

        info_marker = Marker()
        info_marker.header.frame_id = self.rviz_frame_id
        info_marker.header.stamp = stamp
        info_marker.ns = 'system_info'
        info_marker.id = marker_id
        marker_id += 1
        info_marker.type = Marker.TEXT_VIEW_FACING
        info_marker.action = Marker.ADD
        info_marker.pose.position.x = float(rx + 6.0)
        info_marker.pose.position.y = float(ry + 6.0)
        info_marker.pose.position.z = 2.0
        info_marker.pose.orientation.w = 1.0
        info_marker.scale.z = 0.35
        info_marker.color.r = 0.95
        info_marker.color.g = 0.95
        info_marker.color.b = 0.95
        info_marker.color.a = 1.0
        info_marker.text = (
            f"Pozisyon: ({rx:.1f}, {ry:.1f})\n"
            f"Yaw: {math.degrees(r_yaw):.1f}\n"
            f"Toplam: {len(self.detected_objects)} nesne\n"
            f"Yuksek: {high_conf}  Orta: {med_conf}  Dusuk: {low_conf}\n"
            f"{lidar_status}"
        )
        markers.markers.append(info_marker)

        try:
            self.marker_pub.publish(markers)
        except Exception:
            return
    
    def get_buoy_color(self, label):
        """Label'dan renk al - esnek eşleştirme"""
        label_lower = label.lower()
        
        # Önce tam eşleşme
        if label in self.colors:
            return self.colors[label]
        
        # Renk kelimesini içeriyor mu
        color_map = {
            'yellow': (0, 255, 255),
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'black': (100, 100, 100),
            'orange': (0, 165, 255),
            'white': (255, 255, 255)
        }
        
        for color_name, color_val in color_map.items():
            if color_name in label_lower:
                return color_val
        
        return (255, 255, 255)  # Varsayılan beyaz
    
    def get_stable_depth(self, depth_map, u, v, window=3):
        """Daha geniş ROI, daha güvenilir"""
        y1 = max(0, v - window)
        y2 = min(self.image_height, v + window + 1)
        x1 = max(0, u - window)
        x2 = min(self.image_width, u + window + 1)
        
        roi = depth_map[y1:y2, x1:x2]
        valid = roi[(roi > 0.5) & (roi < 20) & np.isfinite(roi)]
        
        if len(valid) < 5:
            return None
        
        median = np.median(valid)
        std = np.std(valid)
        
        # Daha sıkı std kontrolü
        if std > 0.5:
            return None
        
        return float(median)
    
    def pixel_to_global(self, u, v, depth, robot_pose):
        """Piksel -> Global"""
        rx, ry, r_yaw = robot_pose
        
        # Kamera frame
        cam_x = (u - self.image_width / 2) * depth / self.focal_length
        cam_z = depth
        
        # Robot frame
        robot_x = cam_z + self.camera_offset_x
        robot_y = -cam_x
        
        # Global frame
        cos_yaw = math.cos(r_yaw)
        sin_yaw = math.sin(r_yaw)
        
        global_x = rx + (robot_x * cos_yaw - robot_y * sin_yaw)
        global_y = ry + (robot_x * sin_yaw + robot_y * cos_yaw)
        
        return global_x, global_y
    
    def find_closest_object(self, x, y, obj_type):
        """En yakın AYNI TİP nesneyi bul"""
        closest_obj = None
        min_dist = float('inf')
        
        for obj in self.detected_objects:
            if obj['type'] != obj_type:
                continue
            
            dist = math.sqrt((obj['x'] - x)**2 + (obj['y'] - y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_obj = obj
        
        return closest_obj, min_dist
    
    def update_or_add_object(self, global_x, global_y, label):
        """Gelişmiş ekleme/güncelleme"""
        # Buffer sistemi
        grid_size = 3  # Daha küçük grid (daha hassas)
        grid_x = int(global_x / grid_size)
        grid_y = int(global_y / grid_size)
        key = f"{label}_{grid_x}_{grid_y}"
        
        self.detection_buffer[key].append((global_x, global_y))
        
        if len(self.detection_buffer[key]) > self.buffer_size:
            self.detection_buffer[key].pop(0)
        
        # Yeterli tespit yoksa dön
        if len(self.detection_buffer[key]) < self.min_confirmations:
            return
        
        # Median pozisyon
        positions = np.array(self.detection_buffer[key])
        median_x = np.median(positions[:, 0])
        median_y = np.median(positions[:, 1])
        
        # Tutarlılık - daha sıkı
        std = np.std(positions, axis=0)
        if np.max(std) > 1.5:  # 1.5m max sapma
            return
        
        # En yakın aynı tip nesneyi bul
        closest_obj, dist = self.find_closest_object(median_x, median_y, label)
        
        # Birleştirme mesafesi içinde mi?
        if dist < self.merge_distance and closest_obj is not None:
            # GÜNCELLE
            alpha = 0.05  # Çok yavaş güncelleme
            closest_obj['x'] = closest_obj['x'] * (1 - alpha) + median_x * alpha
            closest_obj['y'] = closest_obj['y'] * (1 - alpha) + median_y * alpha
            closest_obj['confidence'] = min(closest_obj['confidence'] + 1, 20)
            closest_obj['last_seen'] = time.time()
        else:
            # YENİ EKLE
            self.detected_objects.append({
                'x': median_x,
                'y': median_y,
                'type': label,
                'color': self.get_buoy_color(label),
                'confidence': self.min_confirmations,
                'last_seen': time.time(),
                'created_at': time.time()
            })
            
            print(f"[+] {label:15s} ({median_x:6.1f}, {median_y:6.1f})")
    
    def lidar_vision_fusion(self, fusion_distance=2.5):
        """
        LIDAR ve Vision füzyonu.
        LIDAR kümesi bir duba ile yakınsa, dubaya ekstra confidence ver.
        """
        if not self.show_lidar or len(self.lidar_clusters) == 0:
            return
        
        for cluster in self.lidar_clusters:
            center = cluster['center']
            
            # Bu kümeye en yakın dubayı bul
            min_dist = float('inf')
            closest_obj = None
            
            for obj in self.detected_objects:
                dist = math.sqrt((obj['x'] - center[0])**2 + (obj['y'] - center[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_obj = obj
            
            # Füzyon mesafesi içindeyse confidence artır
            if min_dist < fusion_distance and closest_obj is not None:
                # LIDAR onayı - daha güvenilir
                closest_obj['confidence'] = min(closest_obj['confidence'] + 0.5, 50)
                closest_obj['lidar_confirmed'] = True
    
    def cleanup_old_objects(self, max_age=30):
        """Uzun süredir görülmeyen düşük güvenli nesneleri sil"""
        current_time = time.time()
        
        before = len(self.detected_objects)
        
        self.detected_objects = [
            obj for obj in self.detected_objects
            if not (obj['confidence'] < 10 and (current_time - obj['last_seen']) > max_age)
        ]
        
        after = len(self.detected_objects)
        
        if before != after:
            print(f"[Temizlik] {before - after} eski nesne silindi")
    
    def harita_ciz(self, robot_pose):
        """Harita görselleştirme"""
        map_img = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)
        cx, cy = self.map_size // 2, self.map_size // 2
        
        # Grid
        grid_spacing = int(5 * self.scale)
        for i in range(0, self.map_size, grid_spacing):
            cv2.line(map_img, (i, 0), (i, self.map_size), (20, 20, 20), 1)
            cv2.line(map_img, (0, i), (self.map_size, i), (20, 20, 20), 1)
        
        # Eksenler
        cv2.line(map_img, (cx, 0), (cx, self.map_size), (0, 60, 0), 2)
        cv2.line(map_img, (0, cy), (self.map_size, cy), (0, 60, 0), 2)
        
        # LIDAR noktaları (alt katman - küçük mavi noktalar)
        if self.show_lidar and len(self.lidar_points) > 0:
            for point in self.lidar_points:
                px = int(cx + point[0] * self.scale)
                py = int(cy - point[1] * self.scale)
                if 0 <= px < self.map_size and 0 <= py < self.map_size:
                    cv2.circle(map_img, (px, py), 1, (100, 50, 50), -1)  # Küçük koyu mavi
        
        # LIDAR kümeleri (daha büyük açık mavi)
        if self.show_lidar:
            for cluster in self.lidar_clusters:
                center = cluster['center']
                px = int(cx + center[0] * self.scale)
                py = int(cy - center[1] * self.scale)
                if 0 <= px < self.map_size and 0 <= py < self.map_size:
                    size = min(cluster['size'] // 3 + 2, 6)
                    cv2.circle(map_img, (px, py), size, (180, 100, 100), -1)  # Açık mavi
                    cv2.circle(map_img, (px, py), size + 1, (200, 150, 150), 1)
        
        # Nesneler - güvene göre sırala (yüksek güven üstte)
        sorted_objects = sorted(self.detected_objects, key=lambda x: x['confidence'])
        
        for obj in sorted_objects:
            px = int(cx + obj['x'] * self.scale)
            py = int(cy - obj['y'] * self.scale)
            
            if not (0 <= px < self.map_size and 0 <= py < self.map_size):
                continue
            
            # Güven seviyesi renk yoğunluğu
            conf_ratio = min(obj['confidence'] / 50, 1.0)
            color = tuple(int(c * (0.3 + 0.7 * conf_ratio)) for c in obj['color'])
            
            # Boyut - güvene göre (max 5 piksel ile sınırlı)
            size = int(3 + min(obj['confidence'] / 15, 2))
            
            # Merge radius göster
            merge_px = int(self.merge_distance * self.scale)
            if obj['confidence'] < 15:  # Düşük güvenli olanlar
                cv2.circle(map_img, (px, py), merge_px, (30, 30, 30), 1)
            
            # Nesne
            cv2.circle(map_img, (px, py), size, color, -1)
            cv2.circle(map_img, (px, py), size + 2, color, 2)
            
            # Güven seviyesi göster (yüksek olanlar için)
            if obj['confidence'] > 20:
                cv2.putText(map_img, str(obj['confidence']), (px + 15, py - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        
        # Robot
        rx, ry, r_yaw = robot_pose
        bot_px = int(cx + rx * self.scale)
        bot_py = int(cy - ry * self.scale)
        
        if 0 <= bot_px < self.map_size and 0 <= bot_py < self.map_size:
            # Gemi boyutları (piksel): uzunluk x genişlik oranı ~2:1
            half_len = 16  # yarı uzunluk (ileri-geri yönü)
            half_wid = 8   # yarı genişlik (yan yönü)
            
            # Yaw açısına göre döndürülmüş dikdörtgen köşeleri
            cos_y = math.cos(r_yaw)
            sin_y = math.sin(r_yaw)
            
            # 4 köşe: ileri-sol, ileri-sağ, geri-sağ, geri-sol
            corners = []
            for (dl, dw) in [(half_len, -half_wid), (half_len, half_wid),
                             (-half_len, half_wid), (-half_len, -half_wid)]:
                rx_off = dl * cos_y - dw * (-sin_y)
                ry_off = dl * (-sin_y) - dw * (-cos_y)
                corners.append([int(bot_px + rx_off), int(bot_py + ry_off)])
            
            pts = np.array(corners, dtype=np.int32)
            
            # Gölge
            pts_shadow = pts + np.array([2, 2], dtype=np.int32)
            cv2.fillPoly(map_img, [pts_shadow], (10, 10, 10))
            
            # Gemi gövdesi
            cv2.fillPoly(map_img, [pts], (120, 120, 255))
            cv2.polylines(map_img, [pts], True, (255, 255, 255), 2)
            
            # Yön oku (burundan ileriye)
            arrow_len = 28
            arrow_start_x = int(bot_px + half_len * cos_y)
            arrow_start_y = int(bot_py - half_len * sin_y)
            end_x = int(bot_px + (half_len + arrow_len) * cos_y)
            end_y = int(bot_py - (half_len + arrow_len) * sin_y)
            cv2.arrowedLine(map_img, (arrow_start_x, arrow_start_y), (end_x, end_y),
                           (0, 0, 255), 3, tipLength=0.35)
        
        # Info panel
        info_h = 180
        cv2.rectangle(map_img, (10, 10), (350, info_h), (12, 12, 12), -1)
        cv2.rectangle(map_img, (10, 10), (350, info_h), (70, 70, 70), 2)
        
        # Güven istatistikleri
        high_conf = sum(1 for o in self.detected_objects if o['confidence'] > 20)
        med_conf = sum(1 for o in self.detected_objects if 10 <= o['confidence'] <= 20)
        low_conf = sum(1 for o in self.detected_objects if o['confidence'] < 10)
        
        lidar_status = f"LIDAR: {len(self.lidar_points)} pkt, {len(self.lidar_clusters)} kume" if self.show_lidar else "LIDAR: KAPALI"
        
        info_texts = [
            f"Pozisyon: ({rx:.1f}, {ry:.1f})",
            f"Yaw: {math.degrees(r_yaw):.1f}",
            f"Toplam: {len(self.detected_objects)} nesne",
            f"Yuksek guven (>20): {high_conf}",
            f"Orta guven (10-20): {med_conf}",
            f"Dusuk guven (<10): {low_conf}",
            lidar_status,
        ]
        
        for i, text in enumerate(info_texts):
            cv2.putText(map_img, text, (20, 35 + i * 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        
        return map_img
    
    def calistir(self):
        """Ana döngü"""
        frame_count = 0
        last_cleanup = time.time()
        
        try:
            while True:
                ret_rgb, frame = self.cam_rgb.get_frame()
                ret_depth, depth_map = self.cam_depth.get_frame()
                robot_pose = self.pozisyon.get_pose()
                
                # LIDAR verilerini al
                if self.lidar.is_ready():
                    self.lidar_points, _ = self.lidar.get_scan_points(robot_pose, self.lidar_max_range)
                    self.lidar_clusters = self.lidar.cluster_points(self.lidar_points, cluster_distance=1.5)
                    # Vision-LIDAR füzyonu
                    self.lidar_vision_fusion()
                
                if not ret_rgb:
                    time.sleep(0.01)
                    continue
                
                frame_count += 1
                
                # Her 30 saniyede temizlik
                if time.time() - last_cleanup > 30:
                    self.cleanup_old_objects()
                    last_cleanup = time.time()
                
                # YOLO
                results = self.model.predict(frame, conf=0.55, verbose=False)  # Daha yüksek conf
                
                for r in results[0].boxes:
                    x1, y1, x2, y2 = map(int, r.xyxy[0])
                    label = self.model.names[int(r.cls[0])]
                    
                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2
                    
                    if not ret_depth:
                        continue
                    
                    depth = self.get_stable_depth(depth_map, u, v)
                    if depth is None or not (1.0 < depth < 20):
                        continue
                    
                    # Global koordinat
                    global_x, global_y = self.pixel_to_global(u, v, depth, robot_pose)
                    
                    # Haritaya ekle
                    self.update_or_add_object(global_x, global_y, label)
                    
                    # Görselleştir
                    color = self.colors.get(label, (0, 255, 0))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f"{label.split('_')[0]} {depth:.1f}m",
                               (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # RViz yayınları
                self.publish_rviz(robot_pose)

                # Nav2 obstacle layer yayını
                self.publish_obstacles()

                # CPU'yu yormamak için kısa bekleme
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n[Sistem] Durduruluyor...")
        
        finally:
            if self.rviz_node is not None:
                self.rviz_node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()
            
            # Final rapor
            if self.detected_objects:
                print("\n" + "=" * 75)
                print(f"{'FİNAL HARİTA':^75}")
                print("=" * 75)
                
                # Güvene göre sırala
                sorted_objs = sorted(self.detected_objects, 
                                    key=lambda x: x['confidence'], 
                                    reverse=True)
                
                print(f"{'#':<4} {'Tip':<18} {'X':>8} {'Y':>8} {'Güven':>8} {'Yaş(s)':>10}")
                print("-" * 75)
                
                current_time = time.time()
                for i, obj in enumerate(sorted_objs, 1):
                    age = current_time - obj['created_at']
                    print(f"{i:<4} {obj['type']:<18} {obj['x']:>8.1f} {obj['y']:>8.1f} "
                          f"{obj['confidence']:>8} {age:>10.1f}")
                
                print("=" * 75)
                print(f"Toplam: {len(self.detected_objects)} nesne")
                print(f"Yüksek güven (>20): {sum(1 for o in self.detected_objects if o['confidence'] > 20)}")
                print("=" * 75)


if __name__ == "__main__":
    sistem = MistikHaritaSistemi()
    sistem.calistir()
