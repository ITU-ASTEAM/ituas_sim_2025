# 🦊 Foxglove Bridge Kurulumu ve Çalıştırılması

> **Platform:** ROS2 Humble | **Amaç:** ROS2 topic'lerini Foxglove Studio üzerinden gerçek zamanlı izlemek

---

## İçindekiler

1. [Kurulum](#1-kurulum)
2. [Bridge'i Başlatma](#2-bridgei-başlatma)
3. [Foxglove Studio ile Bağlantı](#3-foxglove-studio-ile-bağlantı)
4. [Görselleştirilebilecek Veriler](#4-görselleştirilebilecek-veriler)
5. [Bağlantı Doğrulama](#5-bağlantı-doğrulama)

---

## 1. Kurulum

Terminalde aşağıdaki komutu çalıştırın:

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

Bu komut aşağıdaki paketleri otomatik olarak kurar:

| Paket | Açıklama |
|-------|----------|
| `foxglove_bridge` | Ana köprü paketi |
| `rosx_introspection` | ROS mesaj analizi |
| `rapidjson` | JSON serileştirme kütüphanesi |

---

## 2. Bridge'i Başlatma

**Adım 1 —** Workspace dizinine gidin:

```bash
cd ~/Desktop/my_robot_ws
```

**Adım 2 —** Foxglove Bridge'i başlatın:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

**Başarılı çalışma çıktısı:**

```
[INFO] [foxglove_bridge]: Starting foxglove_bridge
[INFO] [foxglove_bridge]: Server listening on port 8765
```

> ✅ Bu mesaj, WebSocket sunucusunun **8765** portunda çalıştığını gösterir.

Bridge çalışırken aşağıdaki topic'ler otomatik olarak yayınlanır:

```
/tf                          # Koordinat dönüşümleri
/map                         # Harita verisi
/odom                        # Odometri
/scan                        # Lidar tarama
/cmd_vel                     # Hız komutları
/oak/rgb/image_raw           # Kamera görüntüsü
/global_costmap/costmap      # Global maliyet haritası
/local_costmap/costmap       # Lokal maliyet haritası
```

---

## 3. Foxglove Studio ile Bağlantı

<!-- 📸 GÖRSEL 1: Buraya Foxglove Studio bağlantı ekranının ekran görüntüsünü ekleyin -->
<!-- Önerilen boyut: 800×500 px -->
<!-- Açıklama: "Connection" > "Open Connection" > "Foxglove WebSocket" seçim ekranı -->

![Foxglove Studio Bağlantı Ekranı](resim1.png)
*Foxglove Studio — Bağlantı oluşturma ekranı*

**Bağlantı adımları:**

1. Foxglove Studio'yu açın
2. **Open Connection** butonuna tıklayın
3. Aşağıdaki ayarları girin:

```
Bağlantı Türü : Foxglove WebSocket
Adres         : ws://localhost:8765
```

4. **Connect** butonuna basın

Bağlantı kurulduktan sonra ROS2 topic'leri Foxglove arayüzünde otomatik olarak listelenir.

---

## 4. Görselleştirilebilecek Veriler

<!-- 📸 GÖRSEL 2: Buraya Foxglove Studio'nun topic görüntüleme arayüzünün ekran görüntüsünü ekleyin -->
<!-- Önerilen boyut: 1200×700 px -->
<!-- Açıklama: Panel'lerin açık olduğu, harita/lidar/kamera verilerinin göründüğü tam arayüz -->

![Foxglove Studio Veri Görselleştirme](resim2.png)
*Foxglove Studio — Gerçek zamanlı navigasyon ve sensör verisi görüntüleme*

| Veri Türü | Topic | Panel Türü |
|-----------|-------|------------|
| Lidar tarama | `/scan` | 3D veya 2D görünüm |
| Harita | `/map` | Map panel |
| Robot konumu | `/odom` | Plot / 3D |
| TF dönüşümleri | `/tf` | 3D görünüm |
| Kamera görüntüsü | `/oak/rgb/image_raw` | Image panel |
| Navigasyon planı | `/plan`, `/local_plan` | 3D görünüm |
| Costmap | `/global_costmap`, `/local_costmap` | Map panel |

---

## 5. Bağlantı Doğrulama

Foxglove Studio başarıyla bağlandığında **terminalde** şu mesajlar görünür:

```
Client ID ... created ROS subscription
```

Bu mesajlar, Foxglove'un ROS topic'lerine abone olduğunu doğrular.

---

## 🔧 Sorun Giderme

**Bağlantı kurulamazsa** port durumunu kontrol edin:

```bash
ss -lntp | grep 8765
```

Beklenen çıktı:

```
LISTEN  0  ...  *:8765  ...
```

> Port listede görünüyorsa bridge düzgün çalışıyor demektir. Görünmüyorsa bridge'i yeniden başlatın.

---

*Son güncelleme: 2025 | ROS2 Humble*
