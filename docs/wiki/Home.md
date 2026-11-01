# 🚢 Otonom Tekne Simülasyon Projesi - Wiki

> **Proje:** Gazebo Sim üzerinde otonom deniz aracı (USV) simülasyonu  
> **Platform:** Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic + VRX  
> **Dil:** Python 3, C++  
> **Maintainer:** BeeQueen Team

---

## 📖 İçindekiler

| Sayfa | Açıklama |
|-------|----------|
| [Ana Sayfa](Home.md) | Proje özeti ve genel bakış |
| [Kurulum ve Çalıştırma](01-Kurulum-ve-Calistirma.md) | Simülasyonu nasıl başlatırsın |
| [Proje Yapısı](02-Proje-Yapisi.md) | Klasör ve dosya haritası |
| [Robot Modeli (SDF)](03-Robot-Modeli.md) | Teknenin fiziksel tanımı, sensörler, motorlar |
| [Dünya Dosyaları (Worlds)](04-Dunya-Dosyalari.md) | Simülasyon dünyaları ve fizik ayarları |
| [Kontrol Sistemi](05-Kontrol-Sistemi.md) | Klavye kontrol, navigasyon, PID |
| [Görüntü İşleme (Vision)](06-Goruntu-Isleme.md) | Kamera, YOLO, derinlik, SLAM haritası |
| [Sensörler (LiDAR & Kamera)](07-Sensorler.md) | Sensör modülleri ve veri akışı |
| [Costmap ve Harita](08-Costmap-Harita.md) | Statik costmap üretimi |
| [Görev Navigasyonu](09-Gorev-Navigasyonu.md) | Waypoint takibi, yarı-otonom görevler |
| [Olası Hatalar ve Çözümler](10-Olasi-Hatalar.md) | Sık karşılaşılan sorunlar |
| [Topic Rehberi](11-Topic-Rehberi.md) | Gazebo Transport topic listesi |

---

## 🏗️ Mimari Genel Bakış

```
┌─────────────────────────────────────────────────────────┐
│                    GAZEBO SIM                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────────┐  │
│  │ Buoyancy │  │ Physics  │  │ VRX Hydrodynamics    │  │
│  │ System   │  │ (DART)   │  │ (Surface + Drag)     │  │
│  └──────────┘  └──────────┘  └──────────────────────┘  │
│                                                         │
│  ┌─────────────── BOAT MODEL ─────────────────────┐    │
│  │  base_link (99.2 kg)                           │    │
│  │  ├── GPU LiDAR (640 ray, 50m range)            │    │
│  │  ├── RGB Camera (640x480, 30fps)               │    │
│  │  ├── Depth Camera (640x480, 30fps)             │    │
│  │  ├── Sol Thruster (leftmotor_link_joint)       │    │
│  │  └── Sağ Thruster (rightmotor_link_joint)      │    │
│  └────────────────────────────────────────────────┘    │
│                                                         │
│  Topics: /lidar, /camera/image_raw,                     │
│          /camera/depth/image_raw, /model/boat/pose      │
└──────────────────┬──────────────────────────────────────┘
                   │ Gazebo Transport (gz.transport13)
                   ▼
┌─────────────────────────────────────────────────────────┐
│                 PYTHON SCRIPTS                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────────┐  │
│  │kontrol.py│  │  nav.py  │  │ withmap.py (SLAM)    │  │
│  │ Klavye   │  │ Waypoint │  │ YOLO + LiDAR + Map   │  │
│  │ Kontrol  │  │ Takip    │  │ Fusion               │  │
│  └──────────┘  └──────────┘  └──────────────────────┘  │
│                                                         │
│  Modüller: kamera_modulu.py, lidar_modulu.py,           │
│            position.py, config.py                       │
└─────────────────────────────────────────────────────────┘
```

---

## ⚡ Hızlı Başlangıç

### İlk Kurulum (Tek Sefer)

> ⚠️ **ÖNEMLİ:** Kuruluma başlamadan önce `setup.py` dosyasının **Masaüstünde** (`~/Desktop/setup.py`) bulunduğundan emin ol! Bu dosya repoda mevcuttur; repoyu klonladıktan sonra `setup.py`'yi Masaüstüne kopyala veya zaten orada olduğunu doğrula.

```bash
# 1. setup.py'nin Masaüstünde olduğundan emin ol
ls ~/Desktop/setup.py   # Bu dosya MUTLAKA olmalı!

# 2. Kurulumu başlat
cd ~/Desktop
sudo python3 setup.py
# Menüden 1'den 7'ye kadar adımları tek tek çalıştır ('A' şu an 6. adımda duruyor)
```

Kurulum bittikten sonra terminali kapat ve yenisini aç.

> Detaylı kurulum rehberi için → [Kurulum ve Çalıştırma](01-Kurulum-ve-Calistirma.md)

### Günlük Kullanım

```bash
# Terminal 1: Simülasyonu başlat
gz sim ~/Desktop/my_robot_ws/src/my_robot_description/worlds/final_boat_hydro.sdf -r

# Terminal 2: Klavye kontrol
cd ~/Desktop/my_robot_ws/scripts
python3 kontrol.py

# Terminal 3: (Opsiyonel) Görüntü işleme + SLAM
cd ~/Desktop/my_robot_ws/scripts/vision
python3 withmap.py
```

---

## 📋 Bağımlılıklar

| Paket | Amaç |
|-------|------|
| `gz-harmonic` (Gazebo Harmonic) | Fizik simülasyonu |
| `gz-transport13` | Python'dan Gazebo iletişimi |
| `gz-msgs10` | Protobuf mesaj tipleri |
| `ros2` Humble Hawksbill | ROS 2 framework (Ubuntu 22.04) |
| `ultralytics` (YOLOv8) | Nesne algılama |
| `depthai` | OAK-D kamera desteği |
| `opencv-python` | Görüntü işleme |
| `numpy` | Sayısal hesaplama |
| `matplotlib` | LiDAR görselleştirme |
| `trimesh`, `pycollada` | Costmap üretimi |
| VRX Plugins (`libSurface.so`, `libSimpleHydrodynamics.so`) | Deniz fiziği |
