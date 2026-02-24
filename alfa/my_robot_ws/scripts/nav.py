#!/usr/bin/env python3
import sys
import time
import math
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_pb2 import Pose

# ==========================================================
# --- AYARLAR ---
# ==========================================================
# Manuel koduna göre: Negatif itki = İLERİ GİDER
FORWARD_THRUST = 80.0       # Hız
TURN_THRUST = 80.0          # Dönüş gücü (Manuel koddaki gibi artırdım)
DISTANCE_TOLERANCE = 1    # 1 metreye girince dur
HEADING_TOLERANCE = 0.2     # ~11 derece hata payı

# Topic'lerin doğru olduğundan emin ol (gz topic -l ile kontrol et)
LEFT_TOPIC = "/model/boat/joint/leftmotor_link_joint/cmd_thrust"
RIGHT_TOPIC = "/model/boat/joint/rightmotor_link_joint/cmd_thrust"
POSE_TOPIC = "/world/ocean_world/model/boat/pose"

class BoatNavigator:
    def __init__(self):
        self.node = Node()
        self.pub_left = self.node.advertise(LEFT_TOPIC, Double)
        self.pub_right = self.node.advertise(RIGHT_TOPIC, Double)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_pose_time = 0  # Son verinin ne zaman geldiğini takip et
        self.pose_received = False

        # Abone ol
        self.node.subscribe(Pose, POSE_TOPIC, self.pose_callback)

        print(f"[{POSE_TOPIC}] dinleniyor... Veri bekleniyor...")
        
        # İlk veriyi bekle (Maksimum 5 saniye)
        start_wait = time.time()
        while not self.pose_received:
            if time.time() - start_wait > 5.0:
                print("\n⚠ HATA: Pose verisi 5 saniyedir gelmiyor!")
                print("  Çözüm: Terminalde 'gz topic -l' yazıp topic adını kontrol et.")
                print("  Çözüm: Simülasyonun 'Play' modunda (akıyor) olduğundan emin ol.")
                sys.exit(1)
            time.sleep(0.1)
        
        print(f"✓ Bağlantı başarılı! Başlangıç: ({self.current_x:.2f}, {self.current_y:.2f})")

    def pose_callback(self, msg):
        try:
            self.current_x = msg.position.x
            self.current_y = msg.position.y
            
            # Quaternion -> Yaw
            qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            self.pose_received = True
            self.last_pose_time = time.time()
            
            # DEBUG: Eğer konumu göremiyorsan aşağıdaki satırın başındaki # işaretini kaldır
            # print(f"DEBUG POSE: x={self.current_x:.2f} y={self.current_y:.2f}")
            
        except Exception as e:
            print(f"Callback Hatası: {e}")

    def publish_thrust(self, left_val, right_val):
        msg_left = Double()
        msg_right = Double()
        msg_left.data = left_val
        msg_right.data = right_val
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

    def stop(self):
        self.publish_thrust(0.0, 0.0)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def navigate_to_point(self, target_x, target_y):
        print(f"\n>>> HEDEF: ({target_x:.2f}, {target_y:.2f}) BAŞLIYOR <<<")
        
        while True:
            # 1. Veri bayatladı mı kontrolü
            if time.time() - self.last_pose_time > 1.0:
                print("\n⚠ UYARI: Pose verisi kesildi! Simülasyon durdu mu?")
                self.stop()
                continue

            # 2. Hesaplamalar
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            bearing = math.atan2(dy, dx)
            heading_error = self.normalize_angle(bearing - self.current_yaw)

            # 3. Durum Yazdır (sys.stdout.flush önemli)
            print(f"\rMesafe: {distance:5.2f}m | Hata: {math.degrees(heading_error):5.1f}° | Konum: ({self.current_x:.1f}, {self.current_y:.1f})   ", end="")
            sys.stdout.flush()

            # 4. Hedefe ulaştı mı?
            if distance < DISTANCE_TOLERANCE:
                self.stop()
                print(f"\n✓ HEDEFE ULAŞILDI. Fark: {distance:.2f}m")
                break

            # 5. Hareket Mantığı
            # Önce yönü düzelt (Olduğu yerde dönme mantığı - Tank Drive)
            if abs(heading_error) > HEADING_TOLERANCE:
                # Yön hatası büyükse, olduğu yerde dön
                if heading_error > 0: 
                    # Sola dön: Sol motor Geri(+), Sağ motor İleri(-)
                    # Manuel kodda 'a' tuşu: Sol=80(Geri), Sağ=-80(İleri)
                    self.publish_thrust(TURN_THRUST, -TURN_THRUST) 
                else: 
                    # Sağa dön
                    self.publish_thrust(-TURN_THRUST, TURN_THRUST)
            else:
                # Yön kabaca doğru, ileri bas
                # Manuel kodda 'w' tuşu: Sol=-60(İleri), Sağ=-60(İleri)
                # Mesafe azaldıkça yavaşla
                speed_scale = min(1.0, distance / 5.0) 
                current_thrust = FORWARD_THRUST * speed_scale
                # İleri gitmek için NEGATİF değer gönderiyoruz
                self.publish_thrust(-current_thrust, -current_thrust)

            time.sleep(0.1) # 10 Hz

def main():
    try:
        nav = BoatNavigator()
        
        # Test rotası
        waypoints = [
            (10, 0),  # 10 metre ileri
            (10, 10), # 10 metre sola (L çiz)
            (0, 0)    # Başlangıca dön
        ]

        print("\nRota yükleniyor...")
        for wp in waypoints:
            nav.navigate_to_point(wp[0], wp[1])
            time.sleep(1)
            
        print("\nGÖREV TAMAMLANDI.")
        
    except KeyboardInterrupt:
        print("\nİptal edildi.")
        nav.stop()

if __name__ == "__main__":
    main()
