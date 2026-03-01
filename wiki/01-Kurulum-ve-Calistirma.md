# 🚀 Kurulum ve Çalıştırma Rehberi

## Ön Gereksinimler

### Sistem Gereksinimleri
- **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS 2:** Humble Hawksbill
- **Gazebo:** Harmonic (`gz-harmonic`)
- **RAM:** Minimum 8 GB (Gazebo + YOLO için 16 GB önerilir)
- **GPU:** NVIDIA GPU önerilir (YOLO ve GPU LiDAR için)

---

## 🔧 Otomatik Kurulum (`setup.py`) — ÖNERİLEN

Tüm kurulum `setup.py` scripti ile otomatik yapılır. **Tek koşul:** bu dosyanın Masaüstünde bulunması.

> 🚨 **UYARI: `setup.py` Masaüstünde (`~/Desktop/setup.py`) OLMALI!**
>
> Kuruluma başlamadan önce dosyanın yerinde olduğunu doğrula:
> ```bash
> ls ~/Desktop/setup.py
> ```
> Eğer dosya yoksa, repodan kopyala:
> ```bash
> cp ~/Desktop/my_robot_ws/setup.py ~/Desktop/setup.py
> ```

### Kullanım

```bash
cd ~/Desktop
sudo python3 setup.py
```

> ⚠️ **SUDO gereklidir** — ROS 2, Gazebo ve sistem paketlerini kurmak için root yetkisi lazım.

### Ana Menü

```
=== GAUSS ROBOTICS SETUP TOOL ===
A. FULL OTOMATİK KURULUM (Adım 1-7)
-----------------------------------
1. Repoları Ekle
2. Paketleri Kur
3. VRX İndir
4. Vision Stack Kur
5. Workspace Derle (Build)
6. İsim/Yol Düzenle
7. Bashrc Kalıcı Yap
8. Simülasyonu Başlat
9. Temel Temizlik (pkill)
10. Sağlam Temizlik (Deep Wipe)
Q. Çıkış
```

### Full Otomatik Kurulum (Önerilen)

Menüden **`A`** seçeneğini gir. Script sırasıyla:

| Adım | İşlem | Açıklama |
|------|-------|----------|
| 1 | Repo Ekleme | ROS 2 apt repo anahtarını ve kaynak listesini ekler |
| 2 | Paket Kurulumu | `ros-humble-desktop`, `gz-harmonic`, `ros-humble-ros-gz`, `python3-colcon-common-extensions` vb. kurar |
| 3 | VRX İndirme | [osrf/vrx](https://github.com/osrf/vrx) reposunu `src/vrx/` altına klonlar |
| 4 | Vision Stack | `ultralytics==8.4.6`, `depthai`, `opencv-python` pip paketlerini kurar |
| 5 | Workspace Build | `colcon build --symlink-install` ile tüm workspace'i derler |
| 6 | İsim/Yol Düzenle | SDF/launch dosyalarındaki `/home/eski_kullanici/` yollarını senin kullanıcı adınla günceller |
| 7 | Bashrc Kalıcı | `.bashrc`'ye ROS 2, workspace source ve `GZ_SIM_*` environment değişkenlerini ekler |

### Kurulum Sonrası Otomatik Eklenen Bashrc Satırları

```bash
# --- ROBOT WS AYARLARI ---
source /opt/ros/humble/setup.bash
source ~/Desktop/my_robot_ws/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/Desktop/my_robot_ws/install/vrx_gz/lib/
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Desktop/my_robot_ws/src/my_robot_description/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Desktop/my_robot_ws/src/my_robot_description/worlds
```

Bu sayede her yeni terminalde otomatik olarak tüm ortam hazır olur.

---

## Adım Adım Manuel Kurulum (Referans)

Setup.py'nin arka planda ne yaptığını bilmek istersen:

### Adım 1: Repoları Ekle
```bash
sudo apt-get update
sudo apt-get install curl wget lsb-release gnupg software-properties-common

# ROS 2 Humble repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
```

### Adım 2: Paketleri Kur
```bash
# ROS 2 + Araçlar
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-pip

# Gazebo Harmonic + Bridge
sudo apt install gz-harmonic ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher
```

### Adım 3: VRX İndir
```bash
cd ~/Desktop/my_robot_ws/src
git clone https://github.com/osrf/vrx.git
```

### Adım 4: Vision Stack
```bash
pip install ultralytics==8.4.6 depthai opencv-python
```

### Adım 5: Workspace Derle
```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/my_robot_ws
colcon build --symlink-install
source install/setup.bash
```

### Adım 6: İsim/Yol Düzenle

SDF ve launch dosyalarındaki hardcoded yollar özgün geliştiricinin kullanıcı adını (`gauss`) içerir. Senin sisteminde çalışması için bu yolların güncellenmesi gerekir.

`setup.py` bunu otomatik yapar (menü seçenek **6**). Elle yapmak istersen:

```bash
# Örnek: /home/gauss/ → /home/senin_adin/
find src/my_robot_description -name "*.sdf" -o -name "*.launch.py" | \
  xargs sed -i 's|/home/gauss/|/home/SENIN_ADIN/|g'
```

> ⚠️ **ÖNEMLİ:** Bu adım yapılmazsa Gazebo mesh/model dosyalarını bulamaz ve tekne görünmez!

### Adım 7: Bashrc Kalıcı Yap
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/Desktop/my_robot_ws/install/setup.bash' >> ~/.bashrc
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/Desktop/my_robot_ws/install/vrx_gz/lib/' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Desktop/my_robot_ws/src/my_robot_description/models' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Desktop/my_robot_ws/src/my_robot_description/worlds' >> ~/.bashrc
```

---

## Simülasyonu Başlatma

### Yöntem 1: Setup.py ile (EN KOLAY)

```bash
cd ~/Desktop
sudo python3 setup.py
# Menüden 8'i seç
```

Bu yöntem:
- Environment değişkenlerini otomatik ayarlar (`GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_SIM_RESOURCE_PATH`)
- ROS 2'yi source eder
- `-r` flag'i ile simülasyonu Play modunda başlatır

### Yöntem 2: Doğrudan Terminal ile

```bash
# Ana simülasyon (ocean_world + parkur + boat)
gz sim ~/Desktop/my_robot_ws/src/my_robot_description/worlds/final_boat_hydro.sdf -r
```

> `-r` flag'i: Simülasyonu başlatıldığında otomatik olarak "Play" moduna alır.

```bash
# Alternatif: Sadece tekne dünyası (parkursuz)
gz sim ~/Desktop/my_robot_ws/install/my_robot_description/share/my_robot_description/worlds/boat_world.sdf -r
```

### Yöntem 3: ROS 2 Launch Dosyası ile

```bash
ros2 launch my_robot_description spawn_boat.launch.py
```

---

## Simülasyon Başlatıldıktan Sonra

### Adım 1: Gazebo'da Play'e Bas
Setup.py veya `-r` flag'i kullandıysan otomatik başlar. Yoksa sol alttaki **▶ Play** butonuna tıkla.

### Adım 2: Topic'leri Kontrol Et
```bash
gz topic -l

# Beklenen topic'ler:
# /lidar
# /camera/image_raw
# /camera/depth/image_raw
# /model/boat/pose
# /model/boat/joint/leftmotor_link_joint/cmd_thrust
# /model/boat/joint/rightmotor_link_joint/cmd_thrust
```

### Adım 3: Kontrol Scriptini Çalıştır
```bash
cd ~/Desktop/my_robot_ws/scripts
python3 kontrol.py
```

### Adım 4: (Opsiyonel) Görüntü İşleme + SLAM
```bash
cd ~/Desktop/my_robot_ws/scripts/goruntuisleme
python3 withmap.py   # Tam SLAM sistemi
# veya
python3 calistir2.py  # Sadece YOLO tespiti
```

---

## Gazebo Temizlik (Donan Simülasyon)

Simülasyon donarsa veya ikinci kez açılmazsa:

### Temel Temizlik (setup.py menü 9)
```bash
pkill -9 -f ign
pkill -9 -f gazebo
pkill -9 -f ruby
```

### Derin Temizlik (setup.py menü 10)
```bash
# Süreçleri öldür
pkill -9 -f 'ign|gazebo|ruby|gz'

# Paylaşımlı hafıza ve geçici dosyaları temizle
rm -rf /dev/shm/ign-* /dev/shm/gz-* /tmp/.ignition /tmp/.gz
```

> 💡 **İPUCU:** Derin temizlik sonrası yeni simülasyon sorunsuz açılır. Eski Gazebo transport layer artıkları temizlenmezse port çakışması yaşanabilir.

---

## Özet Komut Sırası (Copy-Paste)

```bash
# === İLK KURULUM (tek sefer) ===
cd ~/Desktop
sudo python3 setup.py
# Menüden 'A' seçerek full otomatik kurulum yap
# Terminali kapat ve yenisini aç

# === GÜNLÜK KULLANIM ===

# Terminal 1: Simülasyon
gz sim ~/Desktop/my_robot_ws/src/my_robot_description/worlds/final_boat_hydro.sdf -r

# Terminal 2: Kontrol
cd ~/Desktop/my_robot_ws/scripts
python3 kontrol.py

# Terminal 3: Görüntü İşleme (opsiyonel)
cd ~/Desktop/my_robot_ws/scripts/goruntuisleme
python3 withmap.py

# Terminal 4: Navigasyon (opsiyonel)
cd ~/Desktop/my_robot_ws/scripts
python3 nav.py
```

---

## Build Hataları

| Hata | Çözüm |
|------|-------|
| `gz sim` bulunamadı | `sudo python3 setup.py` → menü 2 (Paketleri Kur) |
| `vrx paketleri bulunamadı` | `sudo python3 setup.py` → menü 3 (VRX İndir) |
| `colcon build` başarısız | `sudo python3 setup.py` → menü 5 (Workspace Derle) |
| `libSurface.so not found` | `sudo python3 setup.py` → menü 7 (Bashrc) — Plugin path eksik |
| Mesh/model görünmüyor | `sudo python3 setup.py` → menü 6 (İsim/Yol Düzenle) |
| Simülasyon dondu | `sudo python3 setup.py` → menü 9 veya 10 (Temizlik) |
