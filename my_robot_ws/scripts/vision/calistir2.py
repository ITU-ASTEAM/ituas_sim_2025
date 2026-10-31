#!/usr/bin/env python3

import cv2
import numpy as np
import math
import time
from ultralytics import YOLO
from kamera_modulu import KameraModulu

class MistikVizyonSistemi:
    def __init__(self):
        print("Model yukleniyor: best29jan.pt")
        self.model = YOLO("best29jan.pt")

        print("1. RGB Kamera Bağlanıyor...")
        # 1. RGB Kamera Başlatma
        self.cam_rgb = KameraModulu(topic_name="/camera/image_raw")
        
        print("2. Derinlik Kamerası Bağlanıyor...")
        # 2. Derinlik Kamerası Başlatma (BU SATIR EKSİKTİ)
        self.cam_depth = KameraModulu(topic_name="/camera/depth/image_raw")

        # Kamera Parametreleri
        self.focal_length = 554.25 
        self.image_width = 640

    def calistir(self):
        print("Sistem Aktif. Görüntü bekleniyor...")
        
        try:
            while True:
                # Verileri çek
                ret_rgb, frame = self.cam_rgb.get_frame()
                ret_depth, depth_map = self.cam_depth.get_frame()

                if not ret_rgb:
                    print("RGB veri bekleniyor...", end='\r')
                    time.sleep(0.1)
                    continue

                # Derinlik verisi kontrolü (Debug için)
                if not ret_depth:
                    # Derinlik gelmiyorsa ekrana uyarı bas ama çalışmaya devam et
                    cv2.putText(frame, "Derinlik Yok!", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # YOLO Tahmini
                results = self.model.predict(frame, conf=0.5, verbose=False)

                for r in results[0].boxes:
                    x1, y1, x2, y2 = map(int, r.xyxy[0])
                    label = self.model.names[int(r.cls[0])]
                    
                    # Nesne Merkezi
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    # --- MESAFE OKUMA KISMI ---
                    dist_str = "??"
                    angle_deg = 0.0
                    
                    if ret_depth and depth_map is not None:
                        # Nesne merkezinden 3x3 bir alan al
                        y_min, y_max = max(0, cy-1), min(480, cy+2)
                        x_min, x_max = max(0, cx-1), min(640, cx+2)
                        
                        roi = depth_map[y_min:y_max, x_min:x_max]
                        
                        # Geçerli (sonsuz/NaN olmayan) pikselleri bul
                        valid_pixels = roi[np.isfinite(roi)]
                        
                        if len(valid_pixels) > 0:
                            dist_val = np.min(valid_pixels)
                            # Mesafe 100 metreden büyükse veya 0.1'den küçükse filtrele
                            if 0.1 < dist_val < 100:
                                dist_str = f"{dist_val:.2f}m"
                            else:
                                dist_str = "Hata"

                    # --- AÇI HESABI ---
                    offset_x = cx - (self.image_width / 2)
                    angle_deg = math.degrees(math.atan(offset_x / self.focal_length))

                    # --- ÇİZİM ---
                    color = (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
                    # EKRANA YAZDIRILACAK METİN
                    info_text = f"{label} | {dist_str} | {angle_deg:.1f}dg"
                    
                    # Yazıyı kutunun hemen üstüne yaz
                    cv2.putText(frame, info_text, (x1, y1-15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)

                cv2.imshow("Mistik Vision - Final", frame)
                
                if cv2.waitKey(1) == ord('q'):
                    break

        except KeyboardInterrupt:
            print("\nKapatılıyor...")
        finally:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    vizyon = MistikVizyonSistemi()
    vizyon.calistir()
