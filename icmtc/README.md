# icmtc

Mısır'daki ICMTC yarışmasının test ve yarış günü kodları. Kod arşive olduğu gibi alındı; aşağıda listelenen dosyalar orijinalinde de yoktu. Çalıştırmadan önce bu eksikleri tamamlayın.

## 1. YOLO modeli (`best1JUL.pt`)

| Klasör | Modeli nerede arıyor | Durum |
|---|---|---|
| `gate/primary` | Kendi klasöründe: önce `best1JUL.engine`, yoksa `best1JUL.pt` | İkisi de yok, model yüklenemez |
| `local_map_v3` | Kendi klasöründe: önce `best1JUL.engine`, yoksa `best1JUL.pt` | İkisi de yok, model yüklenemez |
| `local_map_v4_newer_algorithm` | Kendi klasöründe: önce `best1JUL.engine`, yoksa `best1JUL.pt` | `.pt` var; `.engine` olmadığı için yavaş (TensorRT'siz) çalışır |
| `mapping_v2` | Kendi klasöründe: önce `best1JUL.engine`, yoksa `best1JUL.pt` | `.pt` var; `.engine` olmadığı için yavaş (TensorRT'siz) çalışır |
| `fusion_mapping_gps` (`eminfuzyonvf.py:93`) | Komutun çalıştırıldığı klasörde | Script'in yanında yok; `.pt` bulunan bir klasörden çalıştırılmalı |
| `engine_export` (`toengine.py:3`) | Komutun çalıştırıldığı klasörde | Script'in yanında yok; `.pt` bulunan bir klasörden çalıştırılmalı |

**Aynı adla iki farklı model var.** Hangisinin yarışta kullanıldığı bilinmiyor; eksik klasörlere kopyalamadan önce kontrol edin.

| Dosya | Checkpoint içindeki tarih |
|---|---|
| `best1JUL.pt` (bu klasörde) | 2026-07-11 |
| `local_map_v4_newer_algorithm/best1JUL.pt` ve `mapping_v2/best1JUL.pt` (ikisi aynı dosya) | 2026-06-30 |

`local_map_v4_newer_algorithm` yarışmadaki yeni duba renklerine (siyah, mavi) göre yazıldı, ama model bu renklerle yeniden eğitilmedi; ayrıntı `local_map_v4_newer_algorithm/camera_vision_node.py` başındaki notta.

## 2. TensorRT motoru (`best1JUL.engine`)

Hiçbir klasörde yok. `.engine` dosyası kodun çalışacağı cihazda (Jetson Orin NX) `.pt` dosyasından üretilmelidir; başka bir cihazda üretilen dosya çalışmaz (bkz. `engine_export/aciklama.txt`). Üretmek için iki yol var:

- `.pt` dosyasının bulunduğu klasörde `python3 engine_export/toengine.py` (yolu kendi konumunuza göre düzeltin)
- Kodun uyarı mesajındaki komut: `yolo export model=best1JUL.pt format=engine device=0 half=True`

Üretilen `best1JUL.engine` dosyası, onu kullanacak script'in yanına konmalıdır.

## 3. Ayar dosyası (`config.json`)

Aşağıdaki script'ler `config.json` dosyasını kendi klasörlerinde arar. Dosya yoksa uyarı verip koddaki varsayılan ayarlarla (`DEFAULTS`) çalışmaya devam ederler; bu yüzden eksik dosya hata vermez, ama ayarlar sahada kullanılanlar olmayabilir.

| Script | Aradığı dosya | Durum |
|---|---|---|
| `fireson.py` | `config.json` (bu klasörde) | Yok, varsayılanlarla çalışır |
| `water_pump/firesontest.py` | `water_pump/config.json` | Var |
| `water_pump/sds/firefightv2.py`, `water_pump/sds/firesontest_tusdegismis.py` | `water_pump/sds/config.json` | Yok, varsayılanlarla çalışır |

`water_pump/config.json` bu script'lerin beklediği bölümlerin hepsini içeriyor; aynı ayarlarla çalıştırmak için yanlarına kopyalanabilir.

## 4. Repoda olmayan kod

- `local_map_v2_stable`: Klasörde yalnızca `aciklama.txt` var ("jetsonda masaüstünde duruyor"). Kodun kendisi repoya hiç girmedi.
- `calistirmakomutlari.txt`: Yalnızca 3. ve 4. terminalin komutları yazılı (kamera ve lidar sürücüsü). 1. ve 2. terminalde ne çalıştırıldığı yazılmamış.

## Klasör adları

`kamikaze/d` ve `water_pump/sds` yarış sırasında aceleyle açılmış klasörlerdir; adlarının özel bir anlamı yoktur.
