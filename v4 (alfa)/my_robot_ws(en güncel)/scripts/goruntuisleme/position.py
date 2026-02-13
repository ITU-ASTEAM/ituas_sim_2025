#!/usr/bin/env python3
"""
Gazebo'dan robot pozisyonu okuyan modül
Topic: /world/ocean_world/model/boat/pose
"""

import math
import time
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V

class PozisyonModulu:
    def __init__(self, topic_name="/world/ocean_world/model/boat/pose"):
        self.node = Node()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0 
        self.ready = False
        self.last_update_time = 0.0
        
        print(f"[Pozisyon] Dinleniyor: {topic_name}")
        
        # Her iki mesaj tipini de dinle
        if not self.node.subscribe(Pose_V, topic_name, self.callback_pose_v):
            print(f"[Pozisyon] UYARI: Pose_V subscribe başarısız")
        
        if not self.node.subscribe(Pose, topic_name, self.callback_pose):
            print(f"[Pozisyon] UYARI: Pose subscribe başarısız")
        
        # Başlangıç beklemesi
        print("[Pozisyon] İlk veri bekleniyor...")
        timeout = 5.0
        start = time.time()
        
        while not self.ready and (time.time() - start) < timeout:
            time.sleep(0.1)
        
        if self.ready:
            print(f"[Pozisyon] ✓ Hazır! Pos: ({self.x:.2f}, {self.y:.2f}), Yaw: {math.degrees(self.yaw):.1f}°")
        else:
            print(f"[Pozisyon] ✗ HATA: {timeout}s içinde veri gelmedi!")
    
    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Quaternion -> Yaw açısı (radyan)"""
        # Z ekseni rotasyonu (Yaw)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def callback_pose_v(self, msg):
        """Pose_V mesajı geldiğinde (Gazebo genelde bunu kullanır)"""
        if len(msg.pose) == 0:
            return
        
        p = msg.pose[0]
        self.x = p.position.x
        self.y = p.position.y
        self.yaw = self.quaternion_to_yaw(
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w
        )
        self.ready = True
        self.last_update_time = time.time()
    
    def callback_pose(self, msg):
        """Pose mesajı geldiğinde (yedek)"""
        self.x = msg.position.x
        self.y = msg.position.y
        self.yaw = self.quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.ready = True
        self.last_update_time = time.time()
    
    def get_pose(self):
        """Robot pozisyonunu döndür: (x, y, yaw)"""
        return self.x, self.y, self.yaw
    
    def is_ready(self):
        """Veri geldi mi?"""
        return self.ready
    
    def get_age(self):
        """Son güncelleme üzerinden geçen süre (saniye)"""
        if not self.ready:
            return float('inf')
        return time.time() - self.last_update_time


# Test kodu
if __name__ == "__main__":
    print("=" * 60)
    print("POZİSYON MODÜLÜ TESTİ")
    print("=" * 60)
    
    poz = PozisyonModulu()
    
    if not poz.is_ready():
        print("HATA: Pozisyon verisi gelmiyor!")
        exit(1)
    
    print("\nRobotu Gazebo'da hareket ettir...")
    print("Ctrl+C ile çık\n")
    
    try:
        while True:
            x, y, yaw = poz.get_pose()
            yaw_deg = math.degrees(yaw)
            age = poz.get_age()
            
            print(f"Pos: ({x:7.2f}, {y:7.2f}) | Yaw: {yaw:6.3f} rad ({yaw_deg:7.2f}°) | Veri yaşı: {age:.2f}s", end='\r')
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nTest sonlandırıldı.")
