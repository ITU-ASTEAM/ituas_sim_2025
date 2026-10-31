# 📁 Proje Yapısı

## Genel Klasör Haritası

```
my_robot_ws/
├── ../setup.py                   # 🚀 Otomatik kurulum scripti (Masaüstünde)
├── src/                          # 🔧 KAYNAK KODLAR (build öncesi)
│   ├── my_robot_description/     # Robot ve dünya tanımları
│   ├── my_robot_pkg/             # ROS 2 Python paketi
│   ├── hydrodynamic_drag_plugin/ # Custom C++ fizik eklentisi
│   └── vrx/                      # Virtual RobotX (deniz fiziği)
│
├── build/                        # 🏗️ Build çıktıları (otomatik)
├── install/                      # 📦 Kurulum çıktıları (otomatik)
├── log/                          # 📋 Build logları (otomatik)
│
├── scripts/                      # 🐍 ANA PYTHON SCRIPTLERI
│   ├── config.py                 # Topic/isim konfigürasyonu
│   ├── kontrol.py                # Klavye ile tekne kontrolü
│   ├── nav.py                    # Waypoint navigasyonu
│   ├── read_me.txt               # Eski açıklama notu
│   │
│   ├── vision/                   # 👁️ Görüntü işleme modülleri
│   │   ├── kamera_modulu.py      # Kamera veri alıcı (RGB + Depth)
│   │   ├── lidar_modulu.py       # LiDAR veri alıcı
│   │   ├── position.py           # Pozisyon okuyucu modülü
│   │   ├── withmap.py            # ⭐ Ana SLAM sistemi (YOLO+LiDAR+Harita)
│   │   ├── calistir2.py          # Sadece YOLO tespiti (mesafe+açı)
│   │   ├── forrecord.py          # Video kayıt (5dk)
│   │   ├── test.py               # Yaw testi
│   │   ├── best29jan.pt          # YOLO model ağırlıkları
│   │   └── old_weights/          # Yedek model dosyaları
│   │
│   ├── lidar/                    # 📡 Bağımsız LiDAR test
│   │   ├── lidar.py              # Matplotlib ile LiDAR görselleştirme
│   │   └── bridge.yaml           # ROS bridge konfigürasyonu
│   │
│   ├── costmap/                  # 🗺️ Statik costmap üretimi
│   │   ├── dae_to_map.py         # .dae → costmap dönüştürücü
│   │   ├── parkur.dae            # 3D parkur modeli
│   │   ├── colored_map.png       # Üretilmiş harita
│   │   ├── colored_map.yaml      # Harita metadatası
│   │   └── aciklama.txt          # Kullanım notları
│   │
│   ├── missions/                 # 🎯 Görev scriptleri
│   │   └── semi_autonomous/
│   │       └── boat_navigator.py # Waypoint takipli rota navigasyonu
│   │
│   ├── record/                   # 📹 Video kayıt sistemi
│   │   ├── forrecord.py          # Simülasyon video kaydedici
│   │   ├── kamera_modulu.py      # Kamera modülü (kayıt için kopya)
│   │   └── aciklama.txt          # Açıklama
│   │
│   └── untested/                 # 🧪 Test/eski scriptler
│       ├── boat_keyboard.py      # Eski klavye kontrol (subprocess ile)
│       ├── boat_mission.py       # Açık döngü görev (zaman tabanlı)
│       ├── boat_pid_goto.py      # PID ile hedef noktaya gitme
│       └── spawn_boat.launch.py  # Bash ile spawn scripti
│
└── worlds/                       # 🌊 Ekstra dünya dosyaları
    └── ocean.sdf                 # Temel okyanus dünyası
```

---

## Paket Detayları

### `src/my_robot_description/` - Robot Tanımlama Paketi

| Dosya/Klasör | Açıklama |
|---|---|
| `models/boat/model.sdf` | ⭐ Ana tekne SDF modeli (sensörler, motorlar, fizik) |
| `models/boat/meshes/` | 3D mesh dosyaları (STL) |
| `models/itu_lake/` | Gölet parkur modeli |
| `models/teknofest_course/` | Yarışma parkuru modeli |
| `worlds/boat_world.sdf` | Basit su dünyası (test için) |
| `worlds/final_boat_hydro.sdf` | ⭐ Ana simülasyon dünyası (parkur dahil) |
| `launch/spawn_boat.launch.py` | ROS 2 launch: Gazebo + spawn |
| `launch/spawn_robot.launch.py` | ROS 2 launch: Gazebo + URDF + bridge |
| `urdf/robot.urdf` | Eski URDF model (artık SDF kullanılıyor) |
| `CMakeLists.txt` | Build konfigürasyonu |
| `package.xml` | ROS 2 paket tanımlayıcı |

### `src/my_robot_pkg/` - ROS 2 Python Paketi

| Dosya | Açıklama |
|---|---|
| `launch/spawn_boat.launch.py` | Alternatif spawn script (bash) |
| `launch/spawn_my_robot.launch.py` | URDF tabanlı spawn |
| `package.xml` | Python paket manifest |

### `src/hydrodynamic_drag_plugin/` - Custom Fizik Eklentisi

| Dosya | Açıklama |
|---|---|
| `HydrodynamicDrag.cc` | C++ Gazebo plugin (yedek - VRX varken gerekli değil) |
| `CMakeLists.txt` | Plugin build konfigürasyonu |

### `src/vrx/` - Virtual RobotX

VRX (Virtual RobotX) deniz yarışması simülasyon kütüphanesi. Projenin kullandığı kritik eklentiler:
- `libSurface.so` - Su yüzeyi etkileşimi (buoyancy noktaları)
- `libSimpleHydrodynamics.so` - Hidrodinamik sürükleme kuvvetleri

---

## Dosya İlişkileri Diyagramı

```
final_boat_hydro.sdf (dünya)
  ├── includes → models/boat/model.sdf
  │                 ├── Thruster Plugin (sol/sağ motor)
  │                 ├── VRX Surface Plugin (2x pontoon)
  │                 ├── VRX SimpleHydrodynamics Plugin
  │                 ├── PosePublisher Plugin → /model/boat/pose
  │                 ├── GPU LiDAR Sensor → /lidar
  │                 ├── RGB Camera → /camera/image_raw
  │                 └── Depth Camera → /camera/depth/image_raw
  │
  ├── includes → models/teknofest_course/ (engel parkuru)
  ├── Buoyancy System Plugin
  ├── Physics (DART) Plugin
  └── Sensors System Plugin

kontrol.py
  └── gz.transport → /model/boat/joint/leftmotor_link_joint/cmd_thrust
                   → /model/boat/joint/rightmotor_link_joint/cmd_thrust

withmap.py (SLAM)
  ├── kamera_modulu.py → /camera/image_raw, /camera/depth/image_raw
  ├── position.py      → /model/boat/pose
  ├── lidar_modulu.py  → /lidar
  └── YOLO (best29jan.pt) → Nesne tespiti
```
