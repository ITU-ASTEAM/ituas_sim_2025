#!/usr/bin/env python3

import cv2
import time
from kamera_modulu import KameraModulu

print("Kamera başlatılıyor...")
cam = KameraModulu(topic_name="/camera/image_raw")

# Kamera açılsın diye 2 saniye bekle
time.sleep(2)

# İlk frame'i al boyutları öğren
print("İlk görüntü bekleniyor...")
while True:
    ret, frame = cam.get_frame()
    if ret:
        h, w = frame.shape[:2]
        print(f"Görüntü boyutu: {w}x{h}")
        break
    time.sleep(0.1)

# Video kaydedici oluştur
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('egitim_15dk.avi', fourcc, 20.0, (w, h))

print("\n" + "="*50)
print("KAYIT BAŞLADI - 15 DAKİKA")
print("'q' tuşuna basarak durdurabilirsin")
print("="*50 + "\n")

baslangic = time.time()
frame_sayaci = 0
SURE = 900  # 15 dakika = 900 saniye

try:
    while True:
        ret, frame = cam.get_frame()
        
        if not ret:
            print("Frame bekleniyor...", end='\r')
            time.sleep(0.05)
            continue
        
        # Geçen süreyi hesapla
        gecen = time.time() - baslangic
        kalan = SURE - gecen
        
        # Süre doldu mu?
        if gecen >= SURE:
            print(f"\n\n✓ KAYIT TAMAMLANDI! {SURE} saniye doldu")
            break
        
        # Frame'e bilgi yaz
        dakika = int(gecen // 60)
        saniye = int(gecen % 60)
        kalan_dk = int(kalan // 60)
        kalan_sn = int(kalan % 60)
        
        cv2.putText(frame, f"Gecen: {dakika:02d}:{saniye:02d}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Kalan: {kalan_dk:02d}:{kalan_sn:02d}", 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Frame: {frame_sayaci}", 
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Kaydet
        out.write(frame)
        frame_sayaci += 1
        
        # Ekranda göster
        cv2.imshow('Kayit', frame)
        
        # Her 50 frame'de konsola yaz
        if frame_sayaci % 50 == 0:
            print(f"Frame: {frame_sayaci} | Geçen: {dakika:02d}:{saniye:02d} | Kalan: {kalan_dk:02d}:{kalan_sn:02d}", end='\r')
        
        # 'q' tuşuyla çık
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"\n\nKullanıcı tarafından durduruldu")
            break

except KeyboardInterrupt:
    print(f"\n\nCtrl+C ile durduruldu")

finally:
    out.release()
    cv2.destroyAllWindows()
    print(f"\n{'='*50}")
    print(f"Toplam {frame_sayaci} frame kaydedildi")
    print(f"Dosya: egitim_15dk.avi")
    print(f"{'='*50}")
