	#!/usr/bin/env python3
"""
Mistik SLAM v6 - Optimized Version
Daha az tekrar, daha stabil mapping
"""

import cv2
import numpy as np
import math
import time
from collections import defaultdict
from ultralytics import YOLO
from kamera_modulu import KameraModulu
from position import PozisyonModulu

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
        self.model = YOLO("best29jan.pt")
        print("      ✓ Hazır")
        
        print("[2/4] Kameralar...")
        self.cam_rgb = KameraModulu(topic_name="/camera/image_raw")
        self.cam_depth = KameraModulu(topic_name="/camera/depth/image_raw")
        print("      ✓ Hazır")
        
        print("[3/4] Pozisyon...")
        self.pozisyon = PozisyonModulu()
        
        if not self.pozisyon.is_ready():
            print("      ✗ HATA!")
            exit(1)
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
            'orange_buoy': (0, 165, 255)
        }
        
        print("[4/4] Sistem hazır!")
        print("=" * 60)
        print("Tuşlar: q=Çıkış | r=Reset | c=Temizle (confidence<10)")
        print("=" * 60 + "\n")
    
    def get_stable_depth(self, depth_map, u, v, window=3):
        """Daha geniş ROI, daha güvenilir"""
        y1 = max(0, v - window)
        y2 = min(self.image_height, v + window + 1)
        x1 = max(0, u - window)
        x2 = min(self.image_width, u + window + 1)
        
        roi = depth_map[y1:y2, x1:x2]
        valid = roi[(roi > 0.5) & (roi < 45) & np.isfinite(roi)]
        
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
            closest_obj['confidence'] = min(closest_obj['confidence'] + 1, 200)
            closest_obj['last_seen'] = time.time()
        else:
            # YENİ EKLE
            self.detected_objects.append({
                'x': median_x,
                'y': median_y,
                'type': label,
                'color': self.colors.get(label, (255, 255, 255)),
                'confidence': self.min_confirmations,
                'last_seen': time.time(),
                'created_at': time.time()
            })
            
            print(f"[+] {label:15s} ({median_x:6.1f}, {median_y:6.1f})")
    
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
            
            # Boyut - güvene göre
            size = int(3 + min(obj['confidence'] / 8, 12))
            
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
            arrow_len = 40
            end_x = int(bot_px + arrow_len * math.cos(r_yaw))
            end_y = int(bot_py - arrow_len * math.sin(r_yaw))
            
            cv2.circle(map_img, (bot_px + 3, bot_py + 3), 13, (10, 10, 10), -1)
            cv2.circle(map_img, (bot_px, bot_py), 13, (120, 120, 255), -1)
            cv2.arrowedLine(map_img, (bot_px, bot_py), (end_x, end_y),
                           (0, 0, 255), 4, tipLength=0.4)
            cv2.circle(map_img, (bot_px, bot_py), 15, (255, 255, 255), 2)
        
        # Info panel
        info_h = 150
        cv2.rectangle(map_img, (10, 10), (350, info_h), (12, 12, 12), -1)
        cv2.rectangle(map_img, (10, 10), (350, info_h), (70, 70, 70), 2)
        
        # Güven istatistikleri
        high_conf = sum(1 for o in self.detected_objects if o['confidence'] > 20)
        med_conf = sum(1 for o in self.detected_objects if 10 <= o['confidence'] <= 20)
        low_conf = sum(1 for o in self.detected_objects if o['confidence'] < 10)
        
        info_texts = [
            f"Pozisyon: ({rx:.1f}, {ry:.1f})",
            f"Yaw: {math.degrees(r_yaw):.1f}",
            f"Toplam: {len(self.detected_objects)} nesne",
            f"Yuksek guven (>20): {high_conf}",
            f"Orta guven (10-20): {med_conf}",
            f"Dusuk guven (<10): {low_conf}",
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
                    if depth is None or not (1.0 < depth < 35):
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
                
                # Harita
                map_img = self.harita_ciz(robot_pose)
                
                cv2.imshow("Kamera", frame)
                cv2.imshow("Mistik SLAM v6", map_img)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    self.detected_objects.clear()
                    self.detection_buffer.clear()
                    print("[Reset] Harita temizlendi")
                elif key == ord('c'):
                    # Manuel temizlik - confidence < 10 olanları sil
                    before = len(self.detected_objects)
                    self.detected_objects = [o for o in self.detected_objects if o['confidence'] >= 10]
                    after = len(self.detected_objects)
                    print(f"[Temizlik] {before - after} düşük güvenli nesne silindi")
        
        except KeyboardInterrupt:
            print("\n[Sistem] Durduruluyor...")
        
        finally:
            cv2.destroyAllWindows()
            
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
