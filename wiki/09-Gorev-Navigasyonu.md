# 🎯 Görev Navigasyonu

## Genel Bakış

Projede farklı otonom navigasyon yaklaşımları mevcuttur. Basit açık döngüden (open-loop) gelişmiş waypoint takibine kadar çeşitli seviyeler bulunur.

---

## 1. `missions/yarıotonom/boat_navigator.py` ⭐ (En Güncel)

**Konum:** `scripts/missions/yarıotonom/boat_navigator.py`  
**İletişim:** Gazebo Transport (native)  
**Kontrol:** Pulse-based (darbe tabanlı) hareket

### Çalışma Prensibi

Her hareket komutu belirli bir süre uygulanıp durdurulur (pulse):
```
1. Motora itki gönder
2. PULSE_TIME (0.5s) kadar bekle
3. Motorları durdur
4. WAIT_TIME (0.2s) kadar bekle
5. Pozisyonu kontrol et → gerekirse tekrarla
```

### Parametreler

| Parametre | Değer | Açıklama |
|---|---|---|
| `CMD_FORWARD` | -80.0 | İleri itki (negatif = ileri) |
| `CMD_TURN_FORCE` | 60.0 | Dönüş itki gücü |
| `PULSE_TIME` | 0.5 s | Motorun çalışma süresi |
| `WAIT_TIME` | 0.2 s | Bekleme süresi |
| Tolerans | 2.5 m | Waypoint'e yakınlık eşiği |
| Yön eşiği | 0.25 rad (~14°) | Dönüş tetikleme eşiği |

### Waypoint Rotası (Parkur)

```python
WAYPOINTS = [
    (-55.7, 4.2),       # START (Başlangıç noktası)
    (-55.0, -9.7),      # box
    (-44.8, 4.8),       # box_0
    (-37.8, 11.2),      # box_1
    (-32.6, 15.8),      # box_2
    (-24.1, 11.8),      # box_3
    (-12.9, 14.4),      # box_4
    (-2.0, 16.8),       # box_5
    (10.8, 15.7),       # box_6
    (23.2, 13.9),       # box_7
    (33.7, 14.8),       # box_8
    (47.6, 13.6),       # box_9
    (53.9, 15.7),       # box_10
    (60.8, 13.1),       # box_11
    (66.1, 17.7),       # box_12
    (74.8, 15.2),       # box_13
    (83.9, 15.4)        # box_14
]
```

### Görev Aşamaları

```
AŞAMA 1: Başlangıç noktasına git (ilk waypoint, tolerans 1.5m)
         ↓
3 saniye bekleme
         ↓
AŞAMA 2: Rota takibi (waypoint 2 → 15, tolerans 2.5m)
         ↓
GÖREV TAMAMLANDI
```

### Hareket Kontrolü

```python
if abs(angle_diff) > 0.25:
    # Yön sapması büyük → yerinde dön (tank drive)
    if angle_diff > 0:  # SOLA
        pulse(+TURN, -TURN)
    else:               # SAĞA
        pulse(-TURN, +TURN)
else:
    # Yön doğru → ileri git + diferansiyel düzeltme
    correction = angle_diff * 40.0
    pulse(FORWARD + correction, FORWARD - correction)
```

### Thread Safety
Pozisyon modülü callback'lerinde `threading.Lock` kullanılır. Veri okuma ve yazma çakışması önlenir.

---

## 2. `nav.py` - Temel Waypoint Navigasyonu

**Konum:** `scripts/nav.py`

Daha basit, sürekli itki tabanlı navigasyon. Pulse yerine sürekli motor komutu gönderir.

### Farklar (boat_navigator.py'ye göre)

| Özellik | nav.py | boat_navigator.py |
|---|---|---|
| Hareket Tipi | Sürekli itki | Pulse (darbe) |
| Hız Kontrolü | Mesafe oranı (`distance/5.0`) | Sabit itki |
| Tolerans | 1 m | 2.5 m |
| Yön Eşiği | 0.2 rad | 0.25 rad |
| Thread Safety | Yok | Var (Lock) |
| Rota | 3 test noktası | 15+ parkur noktası |

---

## 3. `test edilmeli/boat_mission.py` - Açık Döngü Görev

**Konum:** `scripts/test edilmeli/boat_mission.py`

Feedback olmadan zamana dayalı hareket. Hassas değil ama basit test için uygundur.

### Görev Sırası

```
1. 50m ileri (6 saniye)
2. 90° sola dön (0.8 saniye)
3. 20m ileri (2.4 saniye)
4. 20m geri (2.4 saniye)
5. 270° sağa dön (2.4 saniye)
6. 50m ileri (6 saniye)
```

> ⚠️ Zamana dayalı hareket, akıntı/rüzgar/sürükleme nedeniyle **tutarsız** sonuçlar verir.

---

## 4. `test edilmeli/boat_pid_goto.py` - PID Navigasyonu

**Konum:** `scripts/test edilmeli/boat_pid_goto.py`

En gelişmiş kontrol yaklaşımı ama `subprocess` kullandığı için yavaş.

### PID Parametreleri

| Parametre | Değer | Açıklama |
|---|---|---|
| KP_POS | 1.2 | Pozisyon oransal kazanç |
| KI_POS | 0.05 | Pozisyon integral kazanç |
| KD_POS | 2.5 | Pozisyon türev kazanç |
| KP_TURN | 5.0 | Yön düzeltme kazancı |
| MAX_THRUST | 5.0 | Maksimum itki |
| POS_EPS | 0.1 m | Konum toleransı (±10 cm) |

### Özellikler
- Anti-windup (integral sınırlama: ±10)
- Mesafeye göre yavaşlama (5m'de %50, 2m'de %30)
- Hedef aşımında aktif frenleme
- 2 saniye stabil kalırsa görev tamamlanır

---

## Navigasyon Yöntemi Seçim Rehberi

| Durum | Önerilen Script |
|---|---|
| Parkur yarışması | `boat_navigator.py` |
| Kısa test rotası | `nav.py` |
| Motor/fizik testi | `boat_mission.py` |
| Hassas nokta hedefleme | `boat_pid_goto.py` (refactör gerekli) |
| Manuel keşif | `kontrol.py` |
