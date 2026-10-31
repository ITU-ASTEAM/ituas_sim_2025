# 🎮 Kontrol Sistemi

## Genel Bakış

Projede üç farklı kontrol yaklaşımı mevcuttur:

1. **Klavye Kontrolü** (`kontrol.py`) - Manuel kontrol
2. **Waypoint Navigasyonu** (`nav.py`) - Otonom rota takibi
3. **Eski/Test Scriptleri** (`untested/`) - Alternatif yöntemler

---

## 1. `kontrol.py` - Klavye ile Manuel Kontrol ⭐

**Konum:** `scripts/kontrol.py`  
**İletişim:** Doğrudan Gazebo Transport (`gz.transport13`)

Bu en güncel ve stabil klavye kontrol scriptidir.

### Tuş Atamaları

| Tuş | Eylem | Sol Motor | Sağ Motor |
|-----|-------|-----------|-----------|
| `W` | İleri | -100 (BACK_THRUST) | -100 (BACK_THRUST) |
| `S` | Geri | +100 (FORWARD_THRUST) | +100 (FORWARD_THRUST) |
| `A` | Sola Dön | +40 | -40 |
| `D` | Sağa Dön | -40 | +40 |
| `SPACE` | Durdur | 0 | 0 |
| `Ctrl+C` | Çıkış | 0 | 0 |

> **⚠️ NOT:** Motor değerlerinin işareti ters çalışır! Negatif = ileri, pozitif = geri. Bu SDF modeldeki thruster konfigürasyonundan kaynaklanır.

### Çalıştırma

```bash
cd ~/Desktop/my_robot_ws/scripts
python3 kontrol.py
```

### Nasıl Çalışır

1. `gz.transport13.Node` oluşturur
2. Sol ve sağ motor topic'lerine `Double` mesaj publisher'ı açar
3. Terminal'i raw moduna alarak tek tuşla komut alır
4. Her tuş basımında iki motora itki değeri yayınlar
5. Çıkışta motorları durdurur ve terminal'i eski haline getirir

### Topic'ler

```
/model/boat/joint/leftmotor_link_joint/cmd_thrust   → Double
/model/boat/joint/rightmotor_link_joint/cmd_thrust   → Double
```

---

## 2. `nav.py` - Otonom Waypoint Navigasyonu ⭐

**Konum:** `scripts/nav.py`  
**İletişim:** Doğrudan Gazebo Transport

Kapalı döngü (closed-loop) navigasyon sistemi. Pozisyon feedback'i alarak belirli koordinatlara gider.

### Parametreler

| Parametre | Değer | Açıklama |
|---|---|---|
| `FORWARD_THRUST` | 80.0 | İleri itki gücü |
| `TURN_THRUST` | 80.0 | Dönüş itki gücü |
| `DISTANCE_TOLERANCE` | 1 m | Hedefe yakınlık eşiği |
| `HEADING_TOLERANCE` | 0.2 rad (~11°) | Yön hata payı |

### Çalışma Mantığı

```
1. Pose topic'ine subscribe ol (/world/ocean_world/model/boat/pose)
2. İlk veriyi bekle (5 saniye timeout)
3. Her waypoint için:
   a. Hedef-robot arası mesafe ve açı hesapla
   b. Yön hatası > eşik → olduğu yerde dön (tank drive)
   c. Yön hatası < eşik → ileri git
   d. Mesafe azaldıkça yavaşla (speed_scale = min(1.0, distance/5.0))
   e. Mesafe < 1m → hedefe ulaşıldı, sonraki waypoint'e geç
```

### Varsayılan Test Rotası

```python
waypoints = [
    (10, 0),   # 10m ileri
    (10, 10),  # 10m sola (L çiz)
    (0, 0)     # Başlangıca dön
]
```

### Quaternion → Yaw Dönüşümü

```python
yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
```

---

## 3. Eski/Alternatif Kontrol Scriptleri

### `untested/boat_keyboard.py`
- `subprocess` ile `gz topic` komutu çalıştırarak motor kontrolü yapar
- `kontrol.py`'ye göre **daha yavaş** (her komut ayrı bir process başlatır)
- Trim ayarı desteği var (`+` / `-` tuşları)

### `untested/boat_mission.py`
- **Açık döngü** görev sistemi (feedback yok)
- Zamana dayalı hareket: "6 saniye ileri git, 0.8 saniye sola dön..."
- Hassas kontrol için **uygun değil**, sadece proof-of-concept

### `untested/boat_pid_goto.py`
- PID kontrol ile hedef noktaya gitme
- `subprocess` ile pozisyon okur (JSON parse) ve motor komutları gönderir
- Aktif frenleme ve anti-windup özelliği var
- **Yavaş** çünkü `gz topic -e -n 1` ile her seferinde yeni process başlatır

---

## Kontrol Yöntemleri Karşılaştırma

| Özellik | `kontrol.py` | `nav.py` | `boat_keyboard.py` | `boat_pid_goto.py` |
|---|---|---|---|---|
| İletişim | gz.transport (native) | gz.transport (native) | subprocess (yavaş) | subprocess (yavaş) |
| Feedback | Yok (açık döngü) | Var (kapalı döngü) | Yok | Var (PID) |
| Kullanım | Manuel | Otonom | Manuel | Otonom |
| Hız | ⚡ Anında | ⚡ 10 Hz | 🐌 Yavaş | 🐌 20 Hz (subprocess) |
| Durum | ✅ Güncel | ✅ Güncel | ⚠️ Eski | ⚠️ Eski |

---

## `config.py` - Konfigürasyon Dosyası

Topic isimlerini ve robot parametrelerini merkezi olarak tanımlar:

```python
WORLD_NAME = "ocean_world"
ROBOT_NAME = "boat"
SIM_PREFIX = f"/world/{WORLD_NAME}/model/{ROBOT_NAME}"

class Topics:
    POSE = f"{SIM_PREFIX}/pose"
    LIDAR_SCAN = f"{SIM_PREFIX}/link/lidar_link/sensor/lidar/scan"
    CAM_IMAGE_RAW = f"{SIM_PREFIX}/link/camera_link/sensor/camera/image"
    CMD_VEL = f"/model/{ROBOT_NAME}/cmd_vel"
```

> ⚠️ **NOT:** `config.py` mevcut scriptlerde doğrudan import edilmiyor. Topic isimleri scriptlerde hardcode edilmiş durumda. İleride tüm scriptler bu konfigürasyonu kullanacak şekilde refactör edilebilir.
