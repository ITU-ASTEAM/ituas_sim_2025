# 🌊 Dünya Dosyaları (Worlds)

## Mevcut Dünya Dosyaları

### 1. `final_boat_hydro.sdf` ⭐ (ANA SİMÜLASYON)

**Konum:** `src/my_robot_description/worlds/final_boat_hydro.sdf`  
**Dünya Adı:** `ocean_world`

Bu en güncel ve tam özellikli dünya dosyasıdır. İçerir:

| Özellik | Değer |
|---|---|
| Fizik Motoru | DART (Bullet collision detector ile) |
| Adım Boyutu | 0.004 s |
| Yerçekimi | -9.8 m/s² |
| Su Yoğunluğu | 1000 kg/m³ |
| Su Yüzeyi | 1000 x 1000 m düzlem |
| Gökyüzü | Aktif (saat 12:00) |
| Gölgeler | Aktif |

**Dahil Edilen Modeller:**
- `boat` - Ana tekne (spawn pozisyonu: -55.7, 4.2, 0.3)
- `parkur` - Engel parkuru (pozisyon: 10, 0, 0)

**Aktif Eklentiler:**
- `gz::sim::systems::Physics` - Fizik simülasyonu
- `gz::sim::systems::UserCommands` - Kullanıcı komutları
- `gz::sim::systems::SceneBroadcaster` - Sahne yayını
- `gz::sim::systems::Buoyancy` - Kaldırma kuvveti
- `gz::sim::systems::Sensors` - Sensör sistemi (ogre2 render engine)

**Çalıştırma:**
```bash
gz sim install/my_robot_description/share/my_robot_description/worlds/final_boat_hydro.sdf -r
```

---

### 2. `boat_world.sdf` (Test Dünyası)

**Konum:** `src/my_robot_description/worlds/boat_world.sdf`  
**Dünya Adı:** `boat_world`

Basitleştirilmiş test dünyası. Parkur veya tekne modeli dahil değildir. Tekne spawn komutuyla ayrıca eklenir.

| Özellik | Değer |
|---|---|
| Fizik Motoru | ODE |
| Adım Boyutu | 0.002 s |
| Su Yüzeyi | 200 x 200 m |
| Sensör Sistemi | Yok (sensörler çalışmaz!) |

> ⚠️ **DİKKAT:** Bu dünyada `Sensors` eklentisi tanımlı değil. LiDAR ve kamera verileri **gelmez**. Sadece motor testi için uygundur.

**Çalıştırma:**
```bash
gz sim install/my_robot_description/share/my_robot_description/worlds/boat_world.sdf -r
# Tekneyi ayrıca spawn et:
gz service -s /world/boat_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/path/to/models/boat/model.sdf", name: "boat", pose: {position: {x: 0, y: 0, z: 0.15}}'
```

---

### 3. `ocean.sdf` (Temel Okyanus)

**Konum:** `worlds/ocean.sdf`

Minimal okyanus ortamı. Launch dosyalarından referans edilir ancak güncel kullanımda `final_boat_hydro.sdf` tercih edilmelidir.

---

## Dünya Seçim Rehberi

| Senaryo | Kullanılacak Dünya |
|---|---|
| Tam simülasyon (sensörler + parkur) | `final_boat_hydro.sdf` |
| Sadece motor/hareket testi | `boat_world.sdf` |
| Launch file ile başlatma | `spawn_boat.launch.py` → `boat_world.sdf` |
| Görüntü işleme + LiDAR | **Sadece** `final_boat_hydro.sdf` (Sensors plugin gerekli) |

---

## Parkur ve Gölet Modelleri

### Parkur (`models/parkur/`)
Yarışma engel parkuru. `final_boat_hydro.sdf` dünyasında (10, 0, 0) konumunda dahil edilmiştir.

### Gölet (`models/golet/`)
Alternatif gölet parkuru. Şu an yorumlanmış durumda. Aktifleştirmek için `final_boat_hydro.sdf` içinde yorum kaldırılmalı:

```xml
<!-- Bu satırların yorumunu kaldır: -->
<include>
  <uri>model://golet</uri>
  <name>golet_parkur</name>
  <pose>0 20 -20 +1.5708 0 0</pose>
</include>
```

---

## Fizik Motorları Karşılaştırma

| Özellik | ODE (`boat_world`) | DART (`final_boat_hydro`) |
|---|---|---|
| Kararlılık | Orta | Yüksek |
| Hız | Hızlı | Normal |
| Collision | ODE | Bullet |
| VRX Uyumu | İyi | Çok İyi |
| Önerilen | Test için | **Üretim için** |

> **NOT:** Proje Ubuntu 22.04 üzerinde ROS 2 Humble ve Gazebo Harmonic (`gz-harmonic`) ile çalıştırılmaktadır. Kurulum `setup.py` aracılığıyla otomatik yapılır.
