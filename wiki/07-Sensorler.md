# 📡 Sensör Modülleri

## Genel Bakış

Sensör verilerini Gazebo Transport üzerinden okuyan yeniden kullanılabilir Python modülleri.

---

## `kamera_modulu.py` - Kamera Modülü

**Konum:** `scripts/goruntuisleme/kamera_modulu.py`  
**Sınıf:** `KameraModulu`

Gazebo'daki RGB ve derinlik kameralarından görüntü alan modül.

### Kullanım

```python
from kamera_modulu import KameraModulu

# RGB kamera
cam_rgb = KameraModulu(topic_name="/camera/image_raw")

# Derinlik kamerası
cam_depth = KameraModulu(topic_name="/camera/depth/image_raw")

# Frame al
ret, frame = cam_rgb.get_frame()      # ret: bool, frame: numpy BGR array
ret, depth = cam_depth.get_frame()     # ret: bool, depth: numpy float32 array
```

### Otomatik Mod Algılama

Topic adında `"depth"` kelimesi varsa otomatik olarak derinlik moduna geçer:
- **RGB modu:** `uint8` veri → BGR'ye dönüştürür (3 veya 4 kanal desteği)
- **Derinlik modu:** `float32` veri → 2D array olarak döner (her piksel = metre)

### Callback Mantığı

```
1. Gazebo mesajından width, height, data al
2. Derinlik ise → float32 olarak reshape
3. RGB ise:
   a. uint8 olarak oku
   b. Kanal sayısı hesapla (3 veya 4)
   c. RGBA → BGR veya RGB → BGR dönüşümü
4. self.frame'e kaydet
```

### Hata Koruması
- Kanal sayısı 3 veya 4 değilse frame atlanır
- Tüm hatalar sessizce yakalanır (production mode)

---

## `lidar_modulu.py` - LiDAR Modülü

**Konum:** `scripts/goruntuisleme/lidar_modulu.py`  
**Sınıf:** `LidarModulu`

Gazebo'daki GPU LiDAR sensöründen veri okuyan ve işleyen modül.

### Kullanım

```python
from lidar_modulu import LidarModulu

lidar = LidarModulu(topic_name="/lidar")

# Hazır mı?
if lidar.is_ready():
    # Global koordinatlarda LiDAR noktaları
    points, ranges = lidar.get_scan_points(robot_pose=(x, y, yaw), max_range=20.0)
    
    # Yakın engeller
    obstacles = lidar.get_obstacles(robot_pose, close_threshold=5.0)
    
    # Nokta kümeleme
    clusters = lidar.cluster_points(points, cluster_distance=1.0)
```

### Fonksiyonlar

| Fonksiyon | Girdi | Çıktı | Açıklama |
|---|---|---|---|
| `is_ready()` | - | bool | LiDAR verisi gelmiş mi? |
| `get_scan_points()` | pose, max_range | (N,2) array, ranges | Global koordinatlarda noktalar |
| `get_obstacles()` | pose, threshold | (M,2) array | Yakın engeller |
| `cluster_points()` | points, distance | list[dict] | Küme merkezleri ve boyutları |

### Koordinat Dönüşümü

```
1. Açı dizisi oluştur (angle_min → angle_max)
2. Geçersiz noktaları filtrele (inf, <0.3m, >max_range)
3. Robot frame'e dönüştür:
   x_robot = range * cos(angle) + lidar_offset_x
   y_robot = range * sin(angle)
4. Global frame'e dönüştür:
   x_global = rx + (x_robot * cos(yaw) - y_robot * sin(yaw))
   y_global = ry + (x_robot * sin(yaw) + y_robot * cos(yaw))
```

### LiDAR Offset
- `lidar_offset_x = 0.5` m (robotun merkezinden LiDAR'ın X konumu)
- SDF'deki tanım: `<pose>0.5 0 0.6 0 0 0</pose>`

### Kümeleme Algoritması (Basit)
Greedy nearest-neighbor kümeleme:
1. İlk noktadan başla
2. `cluster_distance` içindeki tüm kullanılmamış noktaları kümeye ekle
3. Küme merkezini hesapla
4. Sonraki kullanılmamış noktaya geç

---

## `position.py` - Pozisyon Modülü

**Konum:** `scripts/goruntuisleme/position.py`  
**Sınıf:** `PozisyonModulu`

Teknenin Gazebo'daki anlık pozisyonunu (x, y, yaw) okuyan modül.

### Kullanım

```python
from position import PozisyonModulu

poz = PozisyonModulu()

if poz.is_ready():
    x, y, yaw = poz.get_pose()
```

### Topic ve Mesaj Tipi

- **Topic:** `/model/boat/pose`
- **Mesaj Tipi:** `Pose_V` (Pose vektörü - birden fazla model pozisyonu içerir)

### Callback Mantığı

```
1. Pose_V mesajından tüm pozisyonları tara
2. "boat" veya "base_link" isimli olanı bul
3. Bulunamazsa ilk pozisyonu al (fallback)
4. Quaternion → Yaw dönüşümü yap
5. x, y, yaw değerlerini güncelle
```

### Beklemeli Başlatma

Modül oluşturulduğunda 5 saniye boyunca ilk veriyi bekler:
```
[Pozisyon] Başlatılıyor...
[Pozisyon] Gazebo'dan ilk veri bekleniyor...
✓ Bağlantı Başarılı!
```

Eğer 5 saniyede veri gelmezse uyarı verir ama çökmez.

---

## `lidar/lidar.py` - Bağımsız LiDAR Görselleştirme

**Konum:** `scripts/lidar/lidar.py`  
**Sınıf:** `LidarPozisyonVisualizer`

Matplotlib ile gerçek zamanlı LiDAR + pozisyon görselleştirmesi. `withmap.py`'den bağımsız çalışır.

### 3 Panelli Görselleştirme

1. **Polar LiDAR Radar** - Kutupsal koordinatlarda tarama
2. **World Map** - Kuş bakışı harita (robot + LiDAR noktaları + yol)
3. **Info Panel** - Sayısal değerler (konum, açı, LiDAR istatistikleri)

### Topic'ler
- Pozisyon: `/world/ocean_world/model/boat/pose` (Pose ve Pose_V her ikisi de dinlenir)
- LiDAR: `/lidar`

---

## `lidar/bridge.yaml` - ROS Bridge Konfigürasyonu

Gazebo LiDAR verisini ROS 2'ye köprüler:

```yaml
- topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "ignition.msgs.LaserScan"
  direction: GZ_TO_ROS
```

Kullanımı (gerektiğinde):
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml
```

---

## Sensör Veri Akış Diyagramı

```
Gazebo Sim
  │
  ├── /lidar (LaserScan) ──────────────► lidar_modulu.py
  │                                         ├── get_scan_points()
  │                                         ├── get_obstacles()
  │                                         └── cluster_points()
  │
  ├── /camera/image_raw (Image) ───────► kamera_modulu.py (RGB mode)
  │                                         └── get_frame() → BGR numpy
  │
  ├── /camera/depth/image_raw (Image) ─► kamera_modulu.py (Depth mode)
  │                                         └── get_frame() → float32 numpy
  │
  └── /model/boat/pose (Pose_V) ───────► position.py
                                            └── get_pose() → (x, y, yaw)
```
