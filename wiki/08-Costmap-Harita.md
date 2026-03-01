# 🗺️ Costmap ve Statik Harita Üretimi

## Genel Bakış

Projede otonom navigasyon için statik costmap üretme altyapısı bulunur. 3D parkur modelinden (.dae dosyası) 2D harita çıkarılır.

---

## `dae_to_map.py` - DAE → Costmap Dönüştürücü

**Konum:** `scripts/costmap/dae_to_map.py`

Collada (.dae) formatındaki 3D parkur modelini 2D renkli harita görüntüsüne dönüştürür.

### Çalıştırma

```bash
cd ~/Desktop/my_robot_ws/scripts/costmap

# Gerekli paketler
pip install trimesh pycollada shapely rtree pillow scipy lxml mapbox_earcut "numpy<2.0"

# Dönüştür
python3 dae_to_map.py
```

### Çıktılar

| Dosya | Açıklama |
|---|---|
| `colored_map.png` | Harita görüntüsü (RGBA) |
| `colored_map.yaml` | Nav2 uyumlu harita metadata |

### YAML Formatı

```yaml
image: colored_map.png
mode: scale
resolution: 0.05          # 1 piksel = 5 cm
origin: [origin_x, origin_y, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Algoritma

```
1. .dae dosyasını trimesh ile yükle
2. Tüm mesh geometrileri ve renklerini çıkar
3. Her yüzey (face) için:
   a. Z koordinatına göre derinlik sıralaması (Painter's Algorithm)
   b. 3D vertex'leri → 2D piksel koordinatlarına dönüştür
   c. Renk bilgisi varsa orijinal rengi kullan, yoksa koyu gri ata
4. PNG + YAML olarak kaydet
```

### Ayarlar

| Parametre | Değer | Açıklama |
|---|---|---|
| `SLICE_HEIGHT_MIN` | 0.05 m | Zemin gürültü filtresi |
| `RESOLUTION` | 0.05 m/px | Harita çözünürlüğü |
| `BACKGROUND_COLOR` | Beyaz (255,255,255) | Boş alan rengi |
| `DEFAULT_OBSTACLE_COLOR` | Koyu gri (50,50,50) | Renksiz engel rengi |

### Hata Yönetimi

- Bozuk mesh index'leri otomatik temizlenir (`mesh.process`)
- Renk okuma hatası → varsayılan gri atanır
- Index out of bounds → sessizce atlanır
- Scene vs Mesh formatı otomatik algılanır

---

## Dosyalar

### `parkur.dae`
Yarışma parkurunun 3D Collada modeli. `dae_to_map.py` bu dosyayı giriş olarak kullanır.

### `colored_map.png`
Üretilmiş 2D harita. Beyaz alan = serbest, renkli/gri alan = engel.

### `colored_map.yaml`
ROS 2 Nav2 costmap sistemiyle uyumlu metadata dosyası.

---

## Nav2 Entegrasyonu (Gelecek)

Bu statik harita Nav2 costmap ile kullanılabilir:

```yaml
# nav2_params.yaml
map_server:
  ros__parameters:
    yaml_filename: "colored_map.yaml"
```

> **NOT:** Şu an proje Gazebo Transport doğrudan kullanıyor. Nav2 entegrasyonu henüz aktif değil ancak altyapı hazır.
