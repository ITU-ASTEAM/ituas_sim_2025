#!/usr/bin/env python3
"""
Gazebo Pozisyon Okuyucu (BEKLEMELİ VERSİYON)
withmap.py için açılışta verinin gelmesini bekler.
"""

import math
import time
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

# Senin sistemindeki doğru topic:
TOPIC_NAME = "/model/boat/pose"

class PozisyonModulu:
    def __init__(self):
        self.node = Node()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0 
        self.ready = False
        self.last_update_time = 0
        
        print(f"[Pozisyon] Başlatılıyor...")
        print(f"[Pozisyon] Hedef Topic: {TOPIC_NAME}")
        
        # Gazebo'dan Pose_V (Vektör) dinliyoruz
        if not self.node.subscribe(Pose_V, TOPIC_NAME, self.callback):
            print(f"[Pozisyon] HATA: Subscribe olunamadı!")

        # --- BURASI YENİ: AÇILIŞTA VERİ GELMESİNİ BEKLE ---
        print("[Pozisyon] Gazebo'dan ilk veri bekleniyor...")
        timeout = 5.0 # 5 saniye bekleme süresi
        start_time = time.time()
        
        while not self.ready:
            if time.time() - start_time > timeout:
                print(f"[Pozisyon] UYARI: {timeout} saniyede veri gelmedi! (Gazebo Play'e bastın mı?)")
                break
            time.sleep(0.1) # İşlemciyi yorma, bekle

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Quaternion -> Yaw dönüşümü"""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def callback(self, msg):
        """Gazebo'dan veri geldiğinde çalışır"""
        target_found = False
        
        for p in msg.pose:
            # İsim kontrolü (boat veya base_link)
            if p.name == "boat" or p.name == "base_link":
                self.x = p.position.x
                self.y = p.position.y
                self.yaw = self.quaternion_to_yaw(
                    p.orientation.x, p.orientation.y,
                    p.orientation.z, p.orientation.w
                )
                self.ready = True
                self.last_update_time = time.time()
                target_found = True
                break
        
        # İsimle bulamazsa ilkini al (Fallback)
        if not target_found and len(msg.pose) > 0:
            p = msg.pose[0]
            self.x = p.position.x
            self.y = p.position.y
            self.yaw = self.quaternion_to_yaw(
                p.orientation.x, p.orientation.y,
                p.orientation.z, p.orientation.w
            )
            self.ready = True
            self.last_update_time = time.time()

    # --- HARİTA MODÜLÜ İÇİN GEREKLİ FONKSİYONLAR ---
    
    def is_ready(self):
        """Harita sistemi bu fonksiyonu çağırıp hazır mı diye sorar"""
        return self.ready

    def get_pose(self):
        """Harita sistemi bu fonksiyonla x, y, yaw çeker"""
        return self.x, self.y, self.yaw

# Test Bloğu (Tek başına çalıştırırsan burası çalışır)
if __name__ == "__main__":
    try:
        poz = PozisyonModulu()
        if poz.is_ready():
            print("✓ Bağlantı Başarılı! Veri akıyor...")
            while True:
                x, y, yaw = poz.get_pose()
                deg = math.degrees(yaw)
                print(f"Pos: ({x:.2f}, {y:.2f}) | Yaw: {deg:.1f}°", end='\r')
                time.sleep(0.1)
        else:
            print("✗ Bağlantı Başarısız. Gazebo Play tuşuna basılı mı?")
    except KeyboardInterrupt:
        print("\nÇıkış.")
