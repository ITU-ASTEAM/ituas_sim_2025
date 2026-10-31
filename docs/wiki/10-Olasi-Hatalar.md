# ⚠️ Olası Hatalar ve Çözümler

## 1. Simülasyon Başlatma Hataları

### ❌ "libSurface.so not found" veya "libSimpleHydrodynamics.so not found"

**Sebep:** VRX eklentileri build edilmemiş veya `GZ_SIM_SYSTEM_PLUGIN_PATH` ayarlanmamış.

**Çözüm:**
```bash
# 1. VRX'i build et
cd ~/Desktop/my_robot_ws
colcon build --packages-select vrx_gz

# 2. Source et
source install/setup.bash

# 3. Plugin path'i kontrol et
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
# Boşsa manuel ekle:
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/Desktop/my_robot_ws/install/vrx_gz/lib
```

### ❌ Simülasyon açılıyor ama tekne yok / tekne uçuyor

**Sebep:** Buoyancy eklentisi yüklenmemiş.

**Çözüm:** `boat_world.sdf` veya `final_boat_hydro.sdf` dosyasında buoyancy plugin'in var olduğundan emin ol:
```xml
<plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
  <uniform_fluid_density>1000</uniform_fluid_density>
</plugin>
```

### ❌ Tekne suya gömülüyor / batıyor

**Sebep:** 
1. Kütle çok fazla veya pontoon boyutları yetersiz
2. VRX Surface plugin aktif değil

**Çözüm:** `model.sdf`'deki kütle ve pontoon parametrelerini kontrol et. Kütle 99.2 kg ve pontoon boyutu 1.15 x 0.22 x 0.20 m olmalı.

### ❌ `gz sim` komutu bulunamadı

**Sebep:** Gazebo Sim kurulu değil.

**Çözüm:**
```bash
# setup.py ile otomatik kurulum (önerilen):
cd ~/Desktop
sudo python3 setup.py   # Menüden 2 seçerek paketleri kur

# Veya manuel:
sudo apt-get install gz-harmonic

# Versiyon kontrol:
gz sim --version
```

---

## 2. Sensör / Topic Hataları

### ❌ Kamera veya LiDAR verisi gelmiyor

**Sebep:** `Sensors` plugini dünya dosyasında tanımlı değil.

**Çözüm:** `boat_world.sdf`'de Sensors eklentisi **YOK**! `final_boat_hydro.sdf` kullan:
```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

### ❌ "Pose verisi 5 saniyede gelmedi"

**Sebep:** 
1. Simülasyon Play modunda değil
2. Topic adı yanlış. Dünya adına göre topic değişir.

**Çözüm:**
```bash
# 1. Gazebo'da Play butonuna bas (veya -r flag'i kullan)

# 2. Aktif topic'leri kontrol et
gz topic -l

# 3. Pose topic'ini dinle
gz topic -e -t /model/boat/pose
```

### ❌ Topic listesinde hiçbir topic yok

**Sebep:** Simülasyon henüz başlatılmamış veya çökmüş.

**Çözüm:** Simülasyonun çalıştığını doğrula. `gz topic -l` komutu en az birkaç system topic göstermeli.

### ❌ LiDAR verisi geliyor ama noktalar yanlış yerde

**Sebep:** LiDAR offset değeri yanlış.

**Çözüm:** `lidar_modulu.py`'deki `lidar_offset_x` değeri SDF'deki LiDAR pozisyonuyla eşleşmeli:
```python
self.lidar_offset_x = 0.5  # SDF: <pose>0.5 0 0.6 0 0 0</pose>
```

---

## 3. Kontrol / Motor Hataları

### ❌ `kontrol.py` çalışıyor ama tekne hareket etmiyor

**Sebep:** 
1. Topic adı yanlış
2. Simülasyon Play modunda değil
3. Terminal aktif pencere değil (tuş girişi algılanmıyor)

**Çözüm:**
```bash
# Topic kontrolü
gz topic -l | grep thrust
# Beklenen:
# /model/boat/joint/leftmotor_link_joint/cmd_thrust
# /model/boat/joint/rightmotor_link_joint/cmd_thrust

# Manuel test (terminalde)
gz topic -t /model/boat/joint/leftmotor_link_joint/cmd_thrust \
  -m gz.msgs.Double -p "data: -50.0"
```

### ❌ Tekne ileri yerine geri gidiyor (veya tam tersi)

**Sebep:** Motor işaret konvansiyonu karışmış. Bu projede **negatif = ileri**.

**Çözüm:** `kontrol.py`'deki değerleri kontrol et:
```python
# İLERİ için negatif değer:
target_left = BACK_THRUST    # -100 (isim yanıltıcı ama negatif = ileri)
target_right = BACK_THRUST   # -100
```

### ❌ Tekne bir tarafa kayıyor (trim problemi)

**Sebep:** Motor kalibrasyonu veya hidrodinamik asimetri.

**Çözüm:** `untested/boat_keyboard.py`'deki trim sistemiyle ayar yap (`+` / `-` tuşları) veya `kontrol.py`'ye trim offset ekle.

---

## 4. Görüntü İşleme Hataları

### ❌ "Model yukleniyor: best29jan.pt" sonrası çökme

**Sebep:** YOLO model dosyası bulunamıyor.

**Çözüm:**
```bash
# Dosyanın mevcut olduğunu kontrol et
ls -la scripts/vision/best29jan.pt

# Script'i doğru dizinden çalıştır
cd scripts/vision
python3 withmap.py  # cwd önemli! YOLO göreceli path kullanıyor
```

### ❌ "RGB veri bekleniyor..." sonsuz döngü

**Sebep:** 
1. Kamera topic'i aktif değil
2. `final_boat_hydro.sdf` yerine `boat_world.sdf` kullanılıyor (Sensors plugin yok)

**Çözüm:** `final_boat_hydro.sdf` ile simülasyonu başlat ve topic'i kontrol et:
```bash
gz topic -l | grep camera
# Beklenen: /camera/image_raw
```

### ❌ Derinlik verisi hep "Hata" veya "??" gösteriyor

**Sebep:** 
1. Derinlik kamerası topic'i aktif değil
2. Nesne çok uzak veya çok yakın

**Çözüm:** Filtreleme aralığını kontrol et:
```python
# calistir2.py / withmap.py
if 0.1 < dist_val < 100:   # Geçerli mesafe aralığı
    dist_str = f"{dist_val:.2f}m"
```

### ❌ Haritadaki nesneler sürekli duplike oluyor

**Sebep:** Birleştirme mesafesi (`merge_distance`) çok küçük.

**Çözüm:** `withmap.py`'de `self.merge_distance` değerini artır:
```python
self.merge_distance = 4.0  # Varsayılan: 4m. Daha büyük yapılabilir.
```

---

## 5. Build Hataları

### ❌ `colcon build` VRX paketinde hata veriyor

**Çözüm:**
```bash
# Kademeli build
colcon build --packages-select my_robot_description
colcon build --packages-select my_robot_pkg
colcon build --packages-select vrx_gz  # VRX'i ayrıca dene
```

### ❌ `gz-transport13` veya `gz-msgs10` Python'da import edilemiyor

**Çözüm:**
```bash
pip install gz-transport13 gz-msgs10

# Veya kaynak Gazebo'nuz ile uyumlu sürümü kontrol edin:
python3 -c "from gz.transport13 import Node; print('OK')"
```

### ❌ `numpy` sürüm uyumsuzluğu (costmap)

**Çözüm:**
```bash
pip install "numpy<2.0"
```

---

## 6. Performans Sorunları

### ❌ Simülasyon çok yavaş / düşük FPS

**Çözümler:**
1. `real_time_factor` değerini artır (hız kazandırır ama gerçekçiliği azaltır)
2. LiDAR örneklem sayısını azalt (640 → 320)
3. Kamera FPS'ini azalt (30 → 15)
4. `ogre2` yerine `ogre` render engine dene

### ❌ Python scriptleri CPU'yu çok kullanıyor

**Çözüm:** Ana döngülerde `time.sleep()` ekle:
```python
time.sleep(0.01)  # 100 Hz maksimum
```

---

### ❌ `source` unutulmuş / ROS 2 Humble ortamı yüklenmemiş

**Sebep:** Her terminalde hem ROS 2 Humble hem de workspace source edilmeli.

**Çözüm:**
```bash
# Her yeni terminalde sırayla çalıştır:
source /opt/ros/humble/setup.bash
source ~/Desktop/my_robot_ws/install/setup.bash

# Otomatikleştirmek için ~/.bashrc'ye ekle:
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/Desktop/my_robot_ws/install/setup.bash' >> ~/.bashrc
```

---

## 7. Yaygın Hatalar Hızlı Referans Tablosu

| Belirti | Olası Sebep | Hızlı Çözüm |
|---|---|---|
| Tekne uçuyor | Buoyancy yok | `final_boat_hydro.sdf` kullan |
| Sensör verisi yok | Sensors plugin yok | `boat_world.sdf` yerine `final_boat_hydro.sdf` |
| Motor çalışmıyor | Play basılmamış | ▶ Play butonuna bas veya `-r` flag |
| Yaw değeri yanlış | Quaternion hatası | `position.py` quaternion_to_yaw kontrol et |
| YOLO bulamıyor | Yanlış dizin | `cd scripts/vision` sonra çalıştır |
| Topic bulunamadı | Dünya adı farklı | `gz topic -l` ile gerçek adı bul |
| Script çöküyor | `source` unutulmuş | `source /opt/ros/humble/setup.bash && source install/setup.bash` |
| Costmap hatalı | numpy ≥ 2.0 | `pip install "numpy<2.0"` |
