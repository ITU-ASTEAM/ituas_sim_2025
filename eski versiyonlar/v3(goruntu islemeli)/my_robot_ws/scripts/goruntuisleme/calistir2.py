#!/usr/bin/env python3
import cv2
import time
from ultralytics import YOLO
from kamera_modulu import KameraModulu
# from hesap_modulu import HesapModulu # (Eğer koordinat hesabı yapacaksan aktif et)

def main():
    # 1. Modeli Yükle (Doğrudan .pt dosyasını kullanıyoruz)
    print("Model yükleniyor: best29jan.pt")
    model = YOLO("best29jan.pt")  # GPU varsa otomatik kullanır

    # 2. Gazebo Kamerasını Başlat
    # DİKKAT: Topic adını 'gz topic -l' ile bulduğunla değiştir!
    cam = KameraModulu(topic_name="/camera/image_raw")
    
    # 3. Hesap Modülü (Gerekirse)
    # hesap = HesapModulu()

    print("Döngü başlıyor...")
    
    try:
        while True:
            # Görüntü al
            ret, frame = cam.get_frame()
            
            if not ret:
                print("Görüntü bekleniyor...", end='\r')
                time.sleep(0.1)
                continue

            # --- YOLO TAHMİNİ ---
            # conf=0.5: Güven eşiği
            # iou=0.45: Çakışma eşiği
            results = model.predict(frame, conf=0.5, iou=0.45, verbose=False)
            
            # Sonuçları işle
            for result in results:
                # Tespit edilen kutuları al (x1, y1, x2, y2, conf, cls)
                boxes = result.boxes.data.cpu().numpy()
                
                for box in boxes:
                    x1, y1, x2, y2, conf, cls = box
                    label = model.names[int(cls)]
                    
                    # Çizim yap
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # --- HESAP MODÜLÜ ENTEGRASYONU ---
                    # OAK-D 'spatial' koordinat (x,y,z metre cinsinden) veriyordu.
                    # Burada sadece piksel koordinatı (x,y) var.
                    # hesap.koordinat_hesapla(x1, y1, x2, y2) gibi bir fonksiyonu buraya bağlayabilirsin.

            # Ekrana bas
            cv2.imshow("Gazebo YOLO Test", frame)

            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nKapatılıyor...")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

