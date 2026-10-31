# 👁️ Görüntü İşleme (Vision) Sistemi

## Genel Bakış

Görüntü işleme sistemi `scripts/vision/` klasöründe yer alır. YOLO nesne algılama, derinlik kamerası ile mesafe ölçümü ve LiDAR ile sensör füzyonu yaparak gerçek zamanlı SLAM haritası oluşturur.

---

## `withmap.py` - Mistik SLAM v6 ⭐ (Ana Sistem)

**Konum:** `scripts/vision/withmap.py`  
**Sınıf:** `MistikHaritaSistemi`

Bu projennin en kapsamlı scriptidir. Tüm sensör verilerini birleştirerek gerçek zamanlı harita oluşturur.

### Kullanılan Modüller
- `kamera_modulu.py` → RGB + Derinlik görüntüsü
- `position.py` → Robot pozisyonu (x, y, yaw)
- `lidar_modulu.py` → LiDAR nokta bulutu
- `ultralytics.YOLO` → Nesne algılama (`best29jan.pt` modeli)

### Başlatma Sırası

```
[1/4] YOLO model yüklenir (best29jan.pt)
[2/4] Kameralar bağlanır (RGB + Derinlik)
[3/5] Pozisyon modülü bağlanır (5 sn timeout)
[4/5] LiDAR modülü bağlanır
[5/5] Sistem hazır!
```

### Ana Döngü

```
Her frame'de:
1. RGB + Depth + Pose + LiDAR verisi al
2. YOLO tespiti yap (conf > 0.55)
3. Her tespit için:
   a. Derinlik haritasından mesafe oku (median, 3x3 ROI)
   b. Piksel → Global koordinata dönüştür
   c. Buffer sistemiyle doğrula (min 6/10 onay)
   d. Mevcut nesneyle birleştir veya yeni ekle
4. LiDAR-Vision füzyonu (2.5m içinde eşleşme)
5. Harita çiz ve göster
6. Her 30 saniyede eski nesneleri temizle
```

### Harita Sistemi

- **Harita Boyutu:** 1200x1200 piksel
- **Ölçek:** 8 piksel/metre
- **Birleştirme Mesafesi:** 4m (aynı tip nesneler)
- **Buffer:** Son 10 tespit, minimum 6 onay gerekli
- **Güven Güncelleme:** α = 0.05 (çok yavaş EMA)

### Tuş Kontrolleri

| Tuş | Eylem |
|-----|-------|
| `q` | Çıkış |
| `r` | Haritayı sıfırla |
| `c` | Düşük güvenli nesneleri temizle (confidence < 10) |
| `l` | LiDAR görselleştirmeyi aç/kapat |

### Piksel → Global Dönüşüm

```python
# 1. Kamera frame → Robot frame
robot_x = depth + camera_offset_x       # İleri mesafe
robot_y = -(u - image_width/2) * depth / focal_length  # Yana mesafe

# 2. Robot frame → Global frame
global_x = rx + (robot_x * cos(yaw) - robot_y * sin(yaw))
global_y = ry + (robot_x * sin(yaw) + robot_y * cos(yaw))
```

### Sensör Füzyonu

LiDAR noktaları kümelenir (1.5m cluster distance). Bir YOLO tespiti ile bir LiDAR kümesi 2.5m içindeyse, nesnenin güven skoru artırılır → "LiDAR confirmed" olur.

### Çıktı (Program Kapanışında)

Program kapandığında tüm tespit edilen nesnelerin listesini yazdırır:
```
# Tip              X        Y    Güven    Yaş(s)
1 red_buoy       12.3     -5.7      35      120.5
2 green_buoy     18.1      3.2      28       89.3
...
```

---

## `calistir2.py` - Basit YOLO Tespiti

**Konum:** `scripts/vision/calistir2.py`  
**Sınıf:** `MistikVizyonSistemi`

Haritasız, sadece YOLO tespiti yapan basit versiyon.

### Özellikler
- RGB kamera + Derinlik kamerası kullanır
- YOLO ile nesne algılar
- Her nesne için:
  - **Mesafe:** Derinlik haritasından 3x3 ROI median
  - **Açı:** `atan(offset_x / focal_length)` ile hesaplanır
- Ekranda bounding box, mesafe ve açı gösterir

### Kamera Parametreleri
- Odak Uzaklığı (focal_length): 554.25 piksel
- Görüntü Genişliği: 640 piksel

---

## `forrecord.py` - Video Kayıt

**Konum:** `scripts/record/forrecord.py`

Simülasyon görüntüsünü AVI formatında kaydeder. YOLO eğitim verisi toplamak için kullanılır.

| Parametre | Değer |
|---|---|
| Format | XVID (.avi) |
| FPS | 20 |
| Süre | 300 saniye (5 dakika) |
| Çözünürlük | Kamera çözünürlüğü (640x480) |

```bash
cd scripts/vision
python3 forrecord.py
# Çıktı: egitim_5dk.avi
```

---

## `test.py` - Yaw Testi

Pozisyon modülünün yaw (sapma açısı) değerinin doğru gelip gelmediğini test eder. Gazebo'da tekneyi döndürüp konsolda açı değişimini gözlemleyebilirsin.

---

## YOLO Model

- **Dosya:** `best29jan.pt` (29 Ocak tarihli eğitim)
- **Framework:** Ultralytics YOLOv8
- **Algılanan Nesne Tipleri:**
  - `yellow_buoy` (Sarı şamandıra)
  - `red_buoy` (Kırmızı şamandıra)
  - `green_buoy` (Yeşil şamandıra)
  - `blue_buoy` (Mavi şamandıra)
  - `black_buoy` (Siyah şamandıra)
  - `orange_buoy` (Turuncu şamandıra)

### Renk Kodları (BGR)

| Nesne | BGR |
|---|---|
| yellow_buoy | (0, 255, 255) |
| red_buoy | (0, 0, 255) |
| green_buoy | (0, 255, 0) |
| blue_buoy | (255, 0, 0) |
| black_buoy | (100, 100, 100) |
| orange_buoy | (0, 165, 255) |
