#!/usr/bin/env python3
"""
MİSTİK VİZYON + KAMİKAZE (ORİJİNAL GÖRSELLEŞTİRME)
- Görüntü işleme ve çizim kodları senin verdiğin orijinal haline sadık kalındı.
- Sadece elde edilen veriler arka planda motor kontrolüne yönlendirildi.
"""

import cv2
import numpy as np
import math
import time
import sys
import threading
from ultralytics import YOLO

# Gazebo Transport ve Kamera Modülü
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from kamera_modulu import KameraModulu

# ============================================================
#  AYARLAR
# ============================================================
YOLO_WEIGHTS  = "best29jan.pt"
CONF_THRESH   = 0.3
YOLO_IMG_SIZE = 416
FOCAL_LENGTH  = 554.25
IMAGE_WIDTH   = 640
IMAGE_HEIGHT  = 480

# Motor Ayarları
LEFT_TOPIC  = "/model/boat/joint/leftmotor_link_joint/cmd_thrust"
RIGHT_TOPIC = "/model/boat/joint/rightmotor_link_joint/cmd_thrust"

CMD_TURN = 40.0      # Arama dönüş hızı
CMD_ATTACK = -100.0  # Dalış hızı
COLLISION_DIST = 2.0 # Çarpma kabul edilecek mesafe (metre)

# ============================================================
#  MOTOR KONTROL MODÜLÜ
# ============================================================
class MotorKontrol:
    def __init__(self):
        self.node = Node()
        self.pub_left = self.node.advertise(LEFT_TOPIC, Double)
        self.pub_right = self.node.advertise(RIGHT_TOPIC, Double)

    def drive(self, left_val, right_val):
        msg_l, msg_r = Double(), Double()
        msg_l.data, msg_r.data = float(left_val), float(right_val)
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def stop(self):
        self.drive(0.0, 0.0)

# ============================================================
#  YOLO WORKER (ORİJİNAL)
# ============================================================
class YoloWorker:
    def __init__(self, model):
        self.model   = model
        self.frame   = None
        self.results = None
        self.lock_in  = threading.Lock()
        self.lock_out = threading.Lock()
        self.running  = True
        self.new_frame = threading.Event()

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, frame):
        with self.lock_in:
            self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.new_frame.set()

    def get_results(self):
        with self.lock_out:
            return self.results

    def _run(self):
        while self.running:
            self.new_frame.wait(timeout=1.0)
            self.new_frame.clear()
            with self.lock_in:
                frame = self.frame
            if frame is None:
                continue
            try:
                res = self.model.predict(frame, conf=CONF_THRESH,
                                         imgsz=YOLO_IMG_SIZE, verbose=False)
                with self.lock_out:
                    self.results = res
            except Exception as e:
                print(f"[YOLO Hata] {e}")

    def stop(self):
        self.running = False

# ============================================================
#  ANA SİSTEM (Vizyon + Otonomi)
# ============================================================
class MistikVizyonSistemi:
    def __init__(self, target_label):
        # Hedef rengi küçük harfe çevir
        self.target_label = target_label.strip().lower()
        
        print("Model yükleniyor:", YOLO_WEIGHTS)
        self.model = YOLO(YOLO_WEIGHTS)

        print("1. RGB Kamera Bağlanıyor...")
        self.cam_rgb   = KameraModulu(topic_name="/camera/image_raw")
        print("2. Derinlik Kamerası Bağlanıyor...")
        self.cam_depth = KameraModulu(topic_name="/camera/depth/image_raw")
        
        print("3. Motor Sürücüleri Bağlanıyor...")
        self.motors = MotorKontrol()

        self.yolo_worker = YoloWorker(self.model)

        self.fps_counter = 0
        self.fps_time    = time.time()
        self.fps_display = 0.0
        
        self.hedef_vuruldu = False

    def _get_distance(self, depth_map, cx, cy):
        if depth_map is None:
            return None
        y1, y2 = max(0, cy-2), min(IMAGE_HEIGHT, cy+3)
        x1, x2 = max(0, cx-2), min(IMAGE_WIDTH,  cx+3)
        roi = depth_map[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi > 0.1) & (roi < 200)]
        if len(valid) == 0:
            return None
        return float(np.min(valid))

    # --- OTONOM SÜRÜŞ MANTIĞI ---
    def otonom_hareket(self, is_target_found, dist, angle):
        if self.hedef_vuruldu: return

        if not is_target_found:
            # Hedef yoksa dönerek ara
            sys.stdout.write(f"\r[ARAMA] {self.target_label.upper()} aranıyor... Dönülüyor.      ")
            sys.stdout.flush()
            self.motors.drive(-CMD_TURN, CMD_TURN)
        else:
            sys.stdout.write(f"\r[KİLİTLENDİ] Mesafe: {dist if dist else 0:.1f}m | Açı: {angle:.1f}°      ")
            sys.stdout.flush()

            # Çarpışma Kontrolü
            if dist is not None and 0.1 < dist < COLLISION_DIST:
                print(f"\n\n💥💥 HEDEF {self.target_label.upper()} VURULDU! 💥💥")
                self.motors.stop()
                self.hedef_vuruldu = True
                return

            # Açıyı Ortalama ve Dalış (Hassasiyet: 8 derece)
            if angle > 8.0:
                self.motors.drive(-CMD_TURN, CMD_TURN)  # Sağa Dön
            elif angle < -8.0:
                self.motors.drive(CMD_TURN, -CMD_TURN)  # Sola Dön
            else:
                self.motors.drive(CMD_ATTACK, CMD_ATTACK) # Merkezde, Tam Gaz İleri

    def calistir(self):
        print(f"\n🚀 SİSTEM AKTİF: [{self.target_label.upper()}] Hedefine Kilitleniliyor.\n")
        time.sleep(2) # Kameraların ısınması için bekle

        try:
            while not self.hedef_vuruldu:
                ret_rgb, frame = self.cam_rgb.get_frame()
                if not ret_rgb:
                    time.sleep(0.05)
                    continue

                ret_depth, depth_map = self.cam_depth.get_frame()
                self.yolo_worker.submit(frame)
                results = self.yolo_worker.get_results()
                draw_frame = frame.copy()

                # Otonomi için anlık veriler
                target_found_this_frame = False
                target_dist = None
                target_angle = 0.0

                if results is not None:
                    for r in results[0].boxes:
                        x1, y1, x2, y2 = map(int, r.xyxy[0])
                        cls_id  = int(r.cls[0])
                        conf    = float(r.conf[0])
                        label   = self.model.names[cls_id].lower() # Küçük harf yap
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                        # Mesafe ve Açı Hesapla
                        dist_val = self._get_distance(depth_map if ret_depth else None, cx, cy)
                        offset_x  = cx - (IMAGE_WIDTH / 2)
                        angle_deg = math.degrees(math.atan(offset_x / FOCAL_LENGTH))

                        # --- OTONOMİ VERİSİ YAKALAMA (Ekrana yansımaz) ---
                        # Eğer bu nesne aradığımız hedefse, verilerini otonomiye gönder
                        if label == self.target_label:
                            target_dist = dist_val
                            target_angle = angle_deg
                            target_found_this_frame = True

                        # --- ORİJİNAL ÇİZİM KODU (DOKUNULMADI) ---
                        color = (0, 255, 0)
                        cv2.rectangle(draw_frame, (x1, y1), (x2, y2), color, 2)
                        
                        dist_str = f"{dist_val:.2f}m" if dist_val else "??"
                        info = f"{label} {conf:.2f} | {dist_str} | {angle_deg:.1f}dg"
                        
                        cv2.putText(draw_frame, info, (x1, max(y1-10, 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
                        cv2.circle(draw_frame, (cx, cy), 4, (0, 0, 255), -1)

                # Motor Komutlarını İşlet
                self.otonom_hareket(target_found_this_frame, target_dist, target_angle)

                # --- FPS ve Ekran Çıktıları (ORİJİNAL) ---
                self.fps_counter += 1
                now = time.time()
                if now - self.fps_time >= 1.0:
                    self.fps_display = self.fps_counter / (now - self.fps_time)
                    self.fps_counter = 0
                    self.fps_time    = now
                
                cv2.putText(draw_frame, f"FPS: {self.fps_display:.1f}", (IMAGE_WIDTH-110, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                cv2.imshow("Mistik Vision - Turbo Kamikaze", draw_frame)
                if cv2.waitKey(1) == ord('q'):
                    break

        except KeyboardInterrupt:
            print("\nAcil durdurma alındı.")
        finally:
            self.motors.stop()
            self.yolo_worker.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    # Kullanıcıdan hedef rengi al
    hedef_renk = input("\nVurulacak dubanın rengini girin (örn: siyah, yesil): ")
    vizyon = MistikVizyonSistemi(hedef_renk)
    vizyon.calistir()
