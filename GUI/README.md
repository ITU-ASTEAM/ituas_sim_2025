# 🖥️ Operatör GUI / İnsan-Robot Arayüzü (HMI)

> **Platform:** ROS2 Humble | **Arayüz:** Foxglove Studio | **Bağlantı:** WebSocket `ws://localhost:8765`

---

## İçindekiler

1. [Genel Bakış](#1-genel-bakış)
2. [Tasarım Tercihleri](#2-tasarım-tercihleri)
3. [Panel Düzeni ve Kullanımı](#3-panel-düzeni-ve-kullanımı)
4. [Gösterilen Veriler ve Gerekçeleri](#4-gösterilen-veriler-ve-gerekçeleri)
5. [Özel Node'lar](#5-özel-nodeler)
6. [Başlatma Prosedürü](#6-başlatma-prosedürü)

---

## 1. Genel Bakış

M�STİK ASV'nin operatör arayüzü, Foxglove Studio'nun açık kaynak WebSocket altyapısı üzerine inşa edilmiş özel bir panel düzenidir. `rosbridge_websocket` üzerinden tüm ROS2 topic'leri gerçek zamanlı olarak görselleştirilir. Teknik olmayan bir operatörün bile batarya durumunu, tekne konumunu, hız ve yön bilgisini tek bakışta okuyabilmesi hedeflenmiştir.

<!-- 📸 GÖRSEL 1: Foxglove Studio genel arayüz görüntüsü -->
<!-- Önerilen boyut: 1400×800 px -->
<!-- Açıklama: Tüm panellerin açık olduğu Overview sekmesi -->
![Foxglove Studio Genel Görünüm](resim1.png)
*Foxglove Studio — MİSTİK ASV operatör arayüzü genel görünüm*

---

## 2. Tasarım Tercihleri

### Neden Foxglove Studio?

Foxglove Studio, ROS2 ekosistemiyle doğrudan entegre olan, açık kaynaklı bir görselleştirme platformudur. Hazır bir HMI sistemi olarak değil, özel panel düzeni ve topic bağlantıları ile yapılandırılmış **açık kaynak bileşen** olarak kullanılmıştır.

| Tercih | Gerekçe |
|--------|---------|
| WebSocket bağlantısı | RFD900 veya WiFi üzerinden uzak erişim imkânı |
| Panel bazlı layout | Her veri türü için en uygun görselleştirme seçilebiliyor |
| Açık kaynak | Lisans maliyeti yok, yarışma ortamında bağımsız çalışır |
| Kayıt özelliği | `.mcap` formatında tüm oturum kaydedilebilir, sonradan analiz edilebilir |

### Layout tasarım felsefesi

Görev sırasında operatörün gözü üç bilgiye öncelikli olarak ihtiyaç duyar: teknenin nerede olduğu, ne yaptığı ve sistemin sağlıklı çalışıp çalışmadığı. Panel yerleşimi bu öncelik sırasına göre düzenlenmiştir:

```
[ Batarya ]  [ Kamera ]           [ Log ]
[      3D Sahne (SLAM + LiDAR)  ] [ Hız / Pusula ]
[ Diagnostics ]                   [ cmd_vel Grafik ]
```

---

## 3. Panel Düzeni ve Kullanımı

### 3.1 Batarya Göstergesi

*Kırmızıdan yeşile renk geçişli dial gauge — tek bakışta güç durumu*

- **Panel tipi:** Gauge (Dial)
- **Topic:** `/battery` → `.percentage`
- **Renk skalası:** Kırmızı (%0–20) → Sarı (%20–60) → Yeşil (%60–100)
- **Aralık:** Min: 0 — Max: 1

### 3.2 Kamera Görüntüsü

- **Panel tipi:** Image
- **Topic:** `/oak/rgb/image_raw`
- **Açıklama:** Teknenin önündeki sahneyi gerçek zamanlı gösterir. Simülasyonda Gazebo kamerasından bridge ile aktarılır; gerçek gemide OAK-D kamera node'u doğrudan bu topic'e publish eder.

### 3.3 Hız ve Pusula Göstergesi


- **Panel tipi:** Raw Messages → `.data` field filtresi
- **Topic:** `/vessel_status`
- **Format:**
```
Heading : 067.3°  NE
Speed   : 0.65 kn
```

### 3.4 3D Sahne

- **Panel tipi:** 3D Scene
- **Fixed Frame:** `map`
- **Bağlı topic'ler:**

| Topic | Görsel Temsil |
|-------|--------------|
| `/mistik_slam/markers` | SLAM haritası ve tekne modeli |
| `/buoys/green` | Yeşil şamandıra nokta bulutu |
| `/buoys/yellow` | Sarı şamandıra nokta bulutu |
| `/scan` | LiDAR tarama verisi |
| `/plan` | Nav2 planlanan rota |
| `/local_costmap/costmap` | Anlık engel haritası |

### 3.5 Diagnostics Paneli

- **Panel tipi:** Diagnostics Summary (ROS)
- **Topic:** `/diagnostics`
- **Gösterilen bilgi:** Nav2 lifecycle manager sağlık durumu — OK / WARN / ERROR renk kodlaması

### 3.6 Motor Komut Grafikleri

- **Panel tipi:** Plot (×2)
- **Topic:** `/cmd_vel` → `linear.x` ve `angular.z`
- **Amaç:** Anlık motor komutlarının düzgünlüğünü ve kararlılığını izlemek

---

## 4. Gösterilen Veriler ve Gerekçeleri

NJORD 2026 teknik inceleme kriterleri doğrultusunda arayüzde bulunması zorunlu veriler ve bunların karşılanma durumu:

| Kriter | Topic | Panel | Durum |
|--------|-------|-------|-------|
| Battery life in % | `/battery` → `.percentage` | Gauge | ✅ |
| Camera | `/oak/rgb/image_raw` | Image | ✅ |
| Heading | `/vessel_status` → `.data` | Raw Messages | ✅ |
| Speed | `/vessel_status` → `.data` | Raw Messages | ✅ |
| Vessel communicates states | `/diagnostics` | Diagnostics | ✅ |
| Route plotted in GUI | `/plan` | 3D Scene | ✅ |

---

## 5. Özel Node'lar

Foxglove'da doğrudan görselleştirilemeyen verileri işlemek için üç özel Python node geliştirilmiştir.

### 5.1 `fake_battery_publisher.py`

`/battery` topic'ine `sensor_msgs/BatteryState` mesajı publish eder. Simülasyonda sanal veri üretir; gerçek gemide fiziksel BMS verisini aynı topic'e aktaran bir node ile yer değiştirecektir. Topic adı sabit kaldığından Foxglove layout'u değiştirilmeyecektir.

```bash
python3 fake_battery_publisher.py
# Opsiyonel parametreler:
# --ros-args -p initial_percentage:=0.90 -p drain_rate:=0.0001 -p voltage:=24.0
```

### 5.2 `vessel_status_publisher.py`

`/odom` topic'ini dinler, iki hesaplama yapar:

- **Hız:** `sqrt(vx² + vy²) × 1.94384` → knot
- **Yön:** Quaternion → `atan2` → derece → 16 noktalı pusula yönü

Sonucu `/vessel_status` topic'ine `std_msgs/String` olarak publish eder.

```bash
python3 vessel_status_publisher.py
```

### 5.3 `gz_camera_bridge.py`

Gazebo simülasyonunda OAK-D kamera doğrudan ROS2'ye mesaj publish etmediğinden, `gz.transport13` kütüphanesi ile Gazebo'nun `/camera/image_raw` topic'i dinlenir ve `sensor_msgs/Image` formatında `/oak/rgb/image_raw`'a köprülenir.

```bash
python3 gz_camera_bridge.py
```

---

## 6. Başlatma Prosedürü

Arayüzü kullanmak için terminalleri sırayla başlatın:

**Terminal 1 — Foxglove Bridge:**
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

**Terminal 2 — Batarya:**
```bash
python3 fake_battery_publisher.py
```

**Terminal 3 — Hız ve Pusula:**
```bash
python3 vessel_status_publisher.py
```

**Terminal 4 — Kamera Bridge:**
```bash
python3 gz_camera_bridge.py
```

**Foxglove Studio:**
```
Bağlantı türü : Foxglove WebSocket
Adres         : ws://localhost:8765
```

> Bağlantı kurulduktan sonra **Overview** sekmesini açın. Tüm paneller otomatik olarak ilgili topic'lere bağlanır.

---

*Son güncelleme: Mayıs 2026 | ROS2 Humble | Foxglove Studio*
