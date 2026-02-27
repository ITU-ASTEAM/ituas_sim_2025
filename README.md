# 🌊 ITU-ASTEAM Gazebo Setup Tool

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Gazebo](https://img.shields.io/badge/Gazebo-Sim-orange) ![Python](https://img.shields.io/badge/Language-Python3-green)

Bu repo, **ITU-ASTEAM** bünyesinde geliştirilen Gazebo simülasyon ortamının ve gerekli bağımlılıkların otomatik olarak kurulmasını sağlayan bir araç setidir.
> [!NOTE]
> İlerleyen süreçte scriptlerin işlevleri ve çalışma mantıkları hakkında daha detaylı teknik dokümantasyon eklenecektir.

## 📂 Önemli Dosyalar ve Veri Setleri (Google Drive)

GitHub'ın dosya boyutu limitleri (100MB) ve veri seti saklama politikaları nedeniyle bazı büyük dosyalar harici olarak sunulmaktadır:

| İçerik | Açıklama | Bağlantı |
| :--- | :--- | :--- |
| **Golet.dae** | 100MB limitini aştığı için Drive üzerinden sunulmaktadır. | [İndir](https://drive.google.com/drive/folders/1548BU8YdyO9oBsV8xFsSzGwSMaZHzrnU?usp=sharing) |
| **Eğitim Datası** | Simülasyon görüntü işleme eğitim veri seti (YOLO/Vision). | [İndir](https://drive.google.com/drive/folders/1XHSol1CxVLURtLInUD2BIfSHCaRn7t5O?usp=drive_link) |


## 🛠️ Ön Hazırlık

Kuruluma başlamadan önce sisteminizde aşağıdaki yapılandırmaların tamamlanmış olduğundan emin olun:

* **Git:** Sisteminizde yüklü olmalıdır.
* **SSH Anahtarı:** GitHub bağlantısı için SSH anahtarınız tanımlanmış olmalıdır.
* **Dizin Yapısı:** `my_robot_ws` dosyanızın **Masaüstünde** (`~/Desktop`) olduğundan emin olun.
* **Temizlik:** `src/` içindeki mevcut `vrx` klasörünü silin (Script güncel repoyu oraya çekecektir).

## 🚀 Kurulum Adımları

Önce repoyu yerel makinenize çekin:

```bash
cd ~/Desktop
git clone https://github.com/ITU-ASTEAM/gazebo
cd gazebo
```

Ardından kurulum scriptini yönetici yetkileriyle başlatın:

```bash
sudo python3 setup.py
```

---

## 🖥️ Setup Tool Kullanımı

Script çalıştığında karşınıza aşağıdaki interaktif menü gelecektir:

| Adım | İşlem | Açıklama |
| :--- | :--- | :--- |
| **1** | **Repoları Ekle** | `tree`, `wget`, `curl` gibi temel araçları yükler. |
| **2** | **Paketleri Kur** | ROS2 Humble ortamını ve bağımlılıklarını hazırlar. |
| **3** | **VRX İndir** | `my_robot_ws/src` altına güncel VRX kütüphanelerini çeker ve derler. |
| **4** | **Vision Stack Kur** | Görüntü işleme araçlarının kurulumunu yapar. |
| **5** | **Workspace Derle** | `colcon build` komutu ile tüm çalışma alanını derler. Ama bu adımdan önce altıncı adımla gerekli dosyaların içeriğindeki bilgisayar isimlerini güncelleyebiliriz |
| **6** | **İsim/Yol Düzenle** | Dosyalardaki kullanıcı adlarını günceller (Örn: `gauss` -> `yeni_isim`). |
| **7** | **Bashrc Kalıcı Yap** | ROS2 ve Gazebo değişkenlerini `.bashrc` dosyasına ekler. |
| **8** | **Simülasyonu Başlat** | Simülasyonu hızlıca çalıştırır. |
| **9** | **Temel Temizlik** | `pkill -9` ile donan `ign`, `gazebo` ve `ruby` süreçlerini temizler. |
| **10** | **Sağlam Temizlik** | **(Deep Wipe)** Süreçlerle birlikte `/dev/shm` ve `/tmp` altındaki hafıza artıklarını uçurur. |
| **Q** | **Çıkış** | Scriptten güvenli çıkış yapar. |

> [!IMPORTANT]
> **Adım 6 Notu:** Bu adımda eski isim kısmına varsayılan olarak `gauss` yazmalısınız. 17 Şubat 2026 itibarıyla sistem, yapıdaki **6 farklı dosyada** otomatik değişiklik yapmaktadır.

---
---

### 📝 Notlar
* VRX kütüphaneleri dış kaynaklı olduğu için güncel repoların çekilmesi, proje bütünlüğü ve hafifliği açısından daha sağlıklıdır. Ayrıca repoyu push ederken vrx'in .git dosyası sebebiyle git bizim repomuza eklemiyor. Sürekli uğraşmamak için o dosya ile uğraşmıyorum.
* Hata almanız durumunda internet bağlantınızı ve dosya izinlerini kontrol ediniz.
  ### 9. Temel Temizlik (pkill)
Simülasyon arayüzü kapansa bile arka planda işlemciyi sömüren fizik motorunu ve ruby scriptlerini zorla kapatır. Port çakışmalarını (`Port already in use`) çözmek için kullanılır.
### 10. Sağlam Temizlik (Deep Wipe)
Süreçlerin yanı sıra, Gazebo'nun paylaşımlı hafızada (Shared Memory) bıraktığı `/dev/shm/gz-*` ve `/tmp/.gz` gibi gizli klasörleri de temizler. **Simülasyon sürekli kilitleniyor veya modeller yüklenmiyorsa bu adımı kullanın.**


## 🎮 Manuel Çalıştırma Komutu
Logları ve hataları temiz takip etmek için şu komut önerilir:
```bash
gz sim -v 4 -r ~/Desktop/my_robot_ws/src/my_robot_description/worlds/final_boat_hydro.sdf
```


---

## 🗺️ Navigasyon ve RViz Çalıştırma (TF Köprüleri)

Simülasyon ayağa kalktıktan sonra otonom sürüş (**Nav2**) özelliklerini kullanmak ve sensör verilerini **RViz** üzerinden mantıksal düzlemde görselleştirmek için aşağıdaki adımları sırasıyla çalıştırmalısınız.

---

### 🚀 1. Navigasyon Stack'ini Başlatma

Teknenin otonom sürüş altyapısını başlatan ana komuttur.  
Sistem konfigürasyonunuza göre RViz arayüzünü de bu komut tetikler.

```bash
ros2 launch boat_navigation boat_nav2.launch.py
```

---

### 🔗 2. Statik Transform (TF) Köprülerinin Kurulması

Gazebo simülasyonundaki fiziksel modelleme ile ROS2'nin mantıksal koordinat sistemi arasında doğrudan bir bağ yoktur.  

RViz'in robotu ve lazer sensör (Lidar) verilerini uzaysal olarak doğru yere konumlandırabilmesi için bir **TF Tree (Transform Ağacı)** oluşturulması şarttır.

> `use_sim_time:=true` parametresi, bu köprülerin simülasyon saati ile senkronize kalmasını sağlar.

---

#### 🔹 A. Gazebo Modeli ile ROS2 `base_link` Köprüsü

Simülasyondaki ana gövde olan `boat` frame'ini, ROS2 navigasyon standartlarında robotun merkezi kabul edilen `base_link` frame’ine sıfır rotasyon ve sıfır öteleme ile bağlar.

```bash
ros2 run tf2_ros static_transform_publisher \
--x 0 --y 0 --z 0 \
--qx 0 --qy 0 --qz 0 --qw 1 \
--frame-id boat \
--child-frame-id base_link \
--ros-args -p use_sim_time:=true
```

---

#### 🔹 B. Lidar Sensörünün Uzaysal Konumlandırması

Lidar verilerinin robotun merkezine (`base_link`) göre nerede bulunduğunu sisteme tanıtır.  

Bu köprü kurulmazsa RViz, lazer noktalarını harita üzerine yerleştirecek referans noktasını bulamaz ve veriler doğru şekilde görselleştirilemez.

```bash
ros2 run tf2_ros static_transform_publisher \
--x 0 --y 0 --z 0 \
--qx 0 --qy 0 --qz 0 --qw 1 \
--frame-id base_link \
--child-frame-id boat/base_link/gpu_lidar \
--ros-args -p use_sim_time:=true
```

---

### ✅ Doğrulama

Bu işlemleri tamamladıktan sonra:

* RViz üzerinde **TF ağacının oluştuğunu**
* `/scan` verisinin aktığını
* Robot modelinin doğru konumda yer aldığını gözlemlemelisiniz.

---
