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
  
## 🚀 Kurulum Adımları

Önce repoyu yerel makinenize çekin:

```bash
cd ~/Desktop
git clone https://github.com/ITU-ASTEAM/ituas_sim_2025.git
cd ituas_sim_2025
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
| **3** | **Vision Stack Kur** | Görüntü işleme araçlarının kurulumunu yapar. |
| **4** | **Workspace Derle** | `colcon build` komutu ile tüm çalışma alanını derler. Ama bu adımdan önce altıncı adımla gerekli dosyaların içeriğindeki bilgisayar isimlerini güncelleyebiliriz |
| **5** | **İsim/Yol Düzenle** | Dosyalardaki kullanıcı adlarını günceller (Örn: `gauss` -> `yeni_isim`). |
| **6** | **Bashrc Kalıcı Yap** | ROS2 ve Gazebo değişkenlerini `.bashrc` dosyasına ekler. |
| **7** | **Simülasyonu Başlat** | Simülasyonu hızlıca çalıştırır. |
| **8** | **Temel Temizlik** | `pkill -9` ile donan `ign`, `gazebo` ve `ruby` süreçlerini temizler. |
| **9** | **Sağlam Temizlik** | **(Deep Wipe)** Süreçlerle birlikte `/dev/shm` ve `/tmp` altındaki hafıza artıklarını uçurur. |
| **Q** | **Çıkış** | Scriptten güvenli çıkış yapar. |

> [!IMPORTANT]
> **Adım 5 Notu:** Bu adımda eski isim kısmına varsayılan olarak `gauss` yazmalısınız. Sistem, yapıdaki **6 farklı dosyada** otomatik değişiklik yapmaktadır.

---
---

### 📝 Notlar
* Hata almanız durumunda internet bağlantınızı ve dosya izinlerini kontrol ediniz.
  ### 8. Temel Temizlik (pkill)
Simülasyon arayüzü kapansa bile arka planda işlemciyi sömüren fizik motorunu ve ruby scriptlerini zorla kapatır. Port çakışmalarını (`Port already in use`) çözmek için kullanılır.
### 9. Sağlam Temizlik (Deep Wipe)
Süreçlerin yanı sıra, Gazebo'nun paylaşımlı hafızada (Shared Memory) bıraktığı `/dev/shm/gz-*` ve `/tmp/.gz` gibi gizli klasörleri de temizler. **Simülasyon sürekli kilitleniyor veya modeller yüklenmiyorsa bu adımı kullanın.**

---

## 🎮 Simülasyon, Navigasyon ve Görüntü İşleme Çalıştırma

Sistemi tam kapasiteyle ayağa kaldırmak için aşağıdaki adımları **farklı terminallerde** sırasıyla çalıştırmalısınız. TF köprüleri sisteme entegre edildiği için manuel kuruluma gerek yoktur.

### 1. Simülasyonu Başlatma
Logları temiz takip etmek ve fizik motorunu ayağa kaldırmak için:
```bash
cd ~/Desktop/my_robot_ws
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
gz sim -v 4 -r ~/Desktop/my_robot_ws/src/my_robot_description/worlds/final_boat_hydro.sdf
```

### 2. Navigasyon Stack (Nav2) ve RViz Başlatma
Teknenin otonom sürüş altyapısını ve RViz arayüzünü başlatmak için:
```bash
cd ~/Desktop/my_robot_ws
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch boat_navigation boat_nav2.launch.py use_sim_time:=true
```
