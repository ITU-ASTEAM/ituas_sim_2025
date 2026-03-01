# 🚤 Robot Modeli (SDF)

## Genel Bakış

Tekne modeli `src/my_robot_description/models/boat/model.sdf` dosyasında tanımlıdır. Model, çift pontonlu (katamaran tipi) bir deniz aracını simüle eder.

## Fiziksel Özellikler

### Ana Gövde (`base_link`)

| Parametre | Değer |
|---|---|
| Kütle | 99.2 kg |
| Ağırlık Merkezi (CoM) | (0.115, 0, -0.485) m |
| Ixx | 0.567 kg·m² |
| Iyy | 1.042 kg·m² |
| Izz | 1.542 kg·m² |

### Pontoon'lar (Collision)

Tekne iki simetrik pontoon (duba) üzerine oturur:

| Özellik | Sol Pontoon | Sağ Pontoon |
|---|---|---|
| Konum Y | +0.314 m | -0.314 m |
| Konum Z | -0.05 m | -0.05 m |
| Boyut | 1.15 x 0.22 x 0.20 m | 1.15 x 0.22 x 0.20 m |

### Motorlar (Thruster)

Tekne iki bağımsız pervane ile sürülür (diferansiyel sürüş - tank drive):

| Özellik | Sol Motor | Sağ Motor |
|---|---|---|
| Joint | `leftmotor_link_joint` | `rightmotor_link_joint` |
| Topic | `/model/boat/joint/leftmotor_link_joint/cmd_thrust` | `/model/boat/joint/rightmotor_link_joint/cmd_thrust` |
| Tip | Revolute | Revolute |
| Pervane Çapı | 0.11 m | 0.11 m |
| Thrust Katsayısı | 0.0001 | 0.0001 |
| Sıvı Yoğunluğu | 1000 kg/m³ | 1000 kg/m³ |
| Kontrol Modu | Velocity control | Velocity control |

**Motor Mantığı:**
- **Negatif değer → İLERİ** hareket
- **Pozitif değer → GERİ** hareket
- Sol ve sağ motorlara farklı değerler vererek dönüş sağlanır

```
İLERİ:  Sol = -80, Sağ = -80
GERİ:   Sol = +80, Sağ = +80
SOLA:   Sol = +60, Sağ = -60  (sol geri, sağ ileri → gemi sola döner)
SAĞA:   Sol = -60, Sağ = +60  (sol ileri, sağ geri → gemi sağa döner)
```

---

## Sensörler

### 1. GPU LiDAR

| Parametre | Değer |
|---|---|
| Tip | `gpu_lidar` |
| Topic | `/lidar` |
| Konum | (0.5, 0, 0.6) m - gövdenin önünde ve üstünde |
| Güncelleme Hızı | 10 Hz |
| Yatay Örneklem | 640 |
| Yatay Açı | -90° ile +90° (-1.5707 ~ 1.5707 rad) |
| Min Mesafe | 0.10 m |
| Max Mesafe | 50.0 m |
| Çözünürlük | 0.01 m |

### 2. RGB Kamera

| Parametre | Değer |
|---|---|
| Tip | `camera` |
| Topic | `/camera/image_raw` |
| Konum | (0.5, 0, 0.4) m |
| Çözünürlük | 640 x 480 |
| Format | R8G8B8 |
| FPS | 30 |
| FOV | 60° (1.047 rad) |
| Clip | 0.1 - 100 m |

### 3. Derinlik Kamerası

| Parametre | Değer |
|---|---|
| Tip | `depth` |
| Topic | `/camera/depth/image_raw` |
| Konum | (0.5, 0, 0.4) m (RGB ile aynı) |
| Çözünürlük | 640 x 480 |
| Format | R_FLOAT32 |
| FPS | 30 |

---

## Eklentiler (Plugins)

### 1. Thruster System (`gz-sim-thruster-system`)
Her motor için ayrı bir thruster plugin'i tanımlı. `cmd_thrust` topic'i üzerinden komut alır.

### 2. VRX Surface (`libSurface.so`)
Her pontoon için ayrı bir yüzey eklentisi. Su seviyesinde kaldırma kuvveti uygular.

```xml
<plugin filename="libSurface.so" name="vrx::Surface">
  <link_name>base_link</link_name>
  <hull_length>1.15</hull_length>
  <hull_radius>0.11</hull_radius>
  <fluid_level>0.0</fluid_level>
  <points>
    <point>0.673604142 0.314 -0.05</point>   <!-- Burun -->
    <point>-0.471395858 0.314 -0.05</point>  <!-- Kıç -->
  </points>
</plugin>
```

### 3. VRX SimpleHydrodynamics (`libSimpleHydrodynamics.so`)
Suyun sürükleme kuvvetlerini simüle eder:

| Parametre | Değer | Açıklama |
|---|---|---|
| `xU` | 100.0 | X yönü lineer sürükleme |
| `xUU` | 150.0 | X yönü karesel sürükleme |
| `yV` | 50.0 | Y yönü lineer sürükleme |
| `yVV` | 100.0 | Y yönü karesel sürükleme |
| `zW` | 200.0 | Z yönü sürükleme |
| `nR` | 30.0 | Yaw rotasyon sürüklemesi |
| `nRR` | 50.0 | Yaw karesel sürükleme |
| `kP` | 12.0 | Roll dengeleme |
| `mQ` | 36.0 | Pitch dengeleme |

### 4. Pose Publisher (`gz-sim-pose-publisher-system`)
Teknenin pozisyon bilgisini yayınlar. `nav.py` ve `position.py` bu veriyi kullanır.

```xml
<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
  <publish_link_pose>true</publish_link_pose>
  <use_pose_vector_msg>true</use_pose_vector_msg>
  <update_frequency>20</update_frequency>
</plugin>
```
Topic: `/model/boat/pose` (Pose_V mesaj tipi)

---

## 3D Görsel (Mesh)

Tekne gövdesi STL mesh dosyaları ile görselleştirilir:
- `denemeas.003.stl` - Ana gövde (2x ölçek)
- `denemeas.001.stl` - Sol pervane
- `denemeas.002.stl` - Sağ pervane

---

## Custom Hydrodynamic Drag Plugin (Yedek)

`src/hydrodynamic_drag_plugin/HydrodynamicDrag.cc` dosyasında VRX'e bağımlı olmayan bir alternatif drag eklentisi mevcuttur. Bu eklenti:
- Lineer ve açısal sürükleme kuvvetleri uygular
- Roll/pitch stabilizasyonu (metacentric height simülasyonu) sağlar
- VRX mevcut olduğunda **kullanılmaya gerek yoktur**
