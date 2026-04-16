#!/usr/bin/env python3
"""
FIXED NAVIGATOR - ÖNCE BAŞLANGIÇ NOKTASINA GİT
"""

import sys
import time
import math
import threading
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V

# ============= AYARLAR =============
# Topicler
LEFT_TOPIC  = "/model/boat/joint/leftmotor_link_joint/cmd_thrust"
RIGHT_TOPIC = "/model/boat/joint/rightmotor_link_joint/cmd_thrust"
POSE_TOPIC = "/world/ocean_world/pose/info"  # ← DOĞRU
MODEL_NAME  = "boat"

# Motor Güçleri (Negatif = İleri, Pozitif = Geri)
CMD_FORWARD = -80.0
CMD_TURN_FORCE = 60.0

# Zamanlamalar
PULSE_TIME = 0.5   # Hareket süresi
WAIT_TIME = 0.2    # Bekleme süresi

# ROTA
WAYPOINTS = [
    (-55.7, 4.2),   # <--- BAŞLANGIÇ NOKTASI (START)
    (-32.165, 16.53),  # box
    (-21.49, 10.284),   # box_0
    (-16.749, 16.50)
    ]

# ============= POZİSYON MODÜLÜ (GZ TRANSPORT) =============

class PozisyonModulu:
    def __init__(self):
        self.node = Node()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0 
        self.ready = False
        self.lock = threading.Lock()
        
        print(f"[Sistem] Pozisyon modülü başlatılıyor...")
        
        if not self.node.subscribe(Pose_V, POSE_TOPIC, self.callback):
            print(f"[HATA] {POSE_TOPIC} topic'ine abone olunamadı!")

        # İlk veri bekleme
        start_wait = time.time()
        while not self.ready:
            if time.time() - start_wait > 5.0:
                print("[UYARI] Veri gelmedi! Simülasyon çalışıyor mu?")
                break
            time.sleep(0.1)

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def callback(self, msg):
        target_found = False
        for p in msg.pose:
            if p.name == "boat" or p.name == MODEL_NAME:
                with self.lock:
                    self.x = p.position.x
                    self.y = p.position.y
                    self.yaw = self.quaternion_to_yaw(
                        p.orientation.x, p.orientation.y,
                        p.orientation.z, p.orientation.w
                    )
                    self.ready = True
                target_found = True
                break
        
        if not target_found and len(msg.pose) > 0:
            p = msg.pose[0]
            with self.lock:
                self.x = p.position.x
                self.y = p.position.y
                self.yaw = self.quaternion_to_yaw(
                    p.orientation.x, p.orientation.y,
                    p.orientation.z, p.orientation.w
                )
                self.ready = True

    def get_pose(self):
        with self.lock:
            return self.x, self.y, self.yaw

# ============= MOTOR KONTROLÜ =============

node = Node()
pub_left = node.advertise(LEFT_TOPIC, Double)
pub_right = node.advertise(RIGHT_TOPIC, Double)

def send_thrust(left_val, right_val):
    msg_left = Double()
    msg_left.data = float(left_val)
    msg_right = Double()
    msg_right.data = float(right_val)
    pub_left.publish(msg_left)
    pub_right.publish(msg_right)

def stop_motors():
    send_thrust(0.0, 0.0)

def move_pulse(left_val, right_val):
    send_thrust(left_val, right_val)
    time.sleep(PULSE_TIME)
    stop_motors()
    time.sleep(WAIT_TIME)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

# ============= HAREKET MANTIĞI =============

def navigate_to_target(poz_modulu, target_x, target_y, tolerance=2.5, message_prefix=""):
    """
    Belirli bir hedefe gidene kadar döngüde kalır.
    True dönerse hedefe ulaştı demektir.
    """
    while True:
        x, y, yaw = poz_modulu.get_pose()
        
        # Mesafe ve Açı Hesabı
        dist = math.sqrt((target_x - x)**2 + (target_y - y)**2)
        target_angle = math.atan2(target_y - y, target_x - x)
        angle_diff = normalize_angle(target_angle - yaw)
        
        # Bilgi Yazdır
        sys.stdout.write(f"\r{message_prefix} | Mesafe: {dist:.1f}m | Açı Farkı: {math.degrees(angle_diff):.1f}°   ")
        sys.stdout.flush()
        
        # Hedefe Vardık mı?
        if dist < tolerance:
            print(f"\n✅ {message_prefix} TAMAMLANDI!")
            stop_motors()
            time.sleep(1.0)
            return True
            
        # Hareket Kontrolü
        if abs(angle_diff) > 0.25: # Yön çok sapmışsa dön
            if angle_diff > 0: 
                # SOLA (Sol+, Sağ-)
                move_pulse(CMD_TURN_FORCE, -CMD_TURN_FORCE)
            else:
                # SAĞA (Sol-, Sağ+)
                move_pulse(-CMD_TURN_FORCE, CMD_TURN_FORCE)
        else:
            # DÜZ GİT (Negatif)
            correction = angle_diff * 40.0
            base_thrust = CMD_FORWARD 
            
            # Differential Drive
            left_cmd = base_thrust + correction 
            right_cmd = base_thrust - correction
            
            move_pulse(left_cmd, right_cmd)

# ============= ANA PROGRAM =============

def main():
    # 1. Modülü Başlat
    poz = PozisyonModulu()
    if not poz.ready:
        print("❌ Pozisyon verisi yok!")
        return

    print("\n" + "="*40)
    print("🚢 GÖREV BAŞLATILIYOR")
    print("="*40)

    # ---------------------------------------------------------
    # AŞAMA 1: BAŞLANGIÇ NOKTASINA GİT (START POINT)
    # ---------------------------------------------------------
    start_x, start_y = WAYPOINTS[0]
    print(f"\n📍 AŞAMA 1: Başlangıç Noktasına Gidiliyor ({start_x}, {start_y})...")
    
    # Toleransı düşük tuttum (1.5m) ki tam otursun
    navigate_to_target(poz, start_x, start_y, tolerance=1.5, message_prefix="[START]")
    
    print("\n⚠️ BAŞLANGIÇ NOKTASINA ULAŞILDI!")
    print("⏳ Görev başlamadan önce bekleniyor...")
    for i in range(3, 0, -1):
        print(f"⏱️  {i}...")
        time.sleep(1)
    
    # ---------------------------------------------------------
    # AŞAMA 2: GÖREV ROTA TAKİBİ (Döngü)
    # ---------------------------------------------------------
    print("\n🚀 AŞAMA 2: ROTA TAKİBİ BAŞLADI!")
    
    # 1. noktadan (start) sonraki noktalara git
    for i in range(1, len(WAYPOINTS)):
        target_x, target_y = WAYPOINTS[i]
        print(f"\n🎯 Hedef {i+1}/{len(WAYPOINTS)}: ({target_x}, {target_y})")
        
        navigate_to_target(poz, target_x, target_y, tolerance=2.5, message_prefix=f"[Waypoint {i+1}]")

    print("\n🏁🏁🏁 TÜM GÖREV BAŞARIYLA TAMAMLANDI! 🏁🏁🏁")
    stop_motors()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 ACİL DURDURMA!")
        stop_motors()
