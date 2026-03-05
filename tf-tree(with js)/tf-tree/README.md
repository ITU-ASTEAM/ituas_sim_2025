# 🌳 ROS2 TF Tree Viewer

Terminal `tree` komutu gibi ROS2 TF frame ağacını **canlı olarak** görselleştiren web uygulaması.

![Demo](./ornek.png)

## Nasıl Çalışır?

Uygulama açıldığında veya "⟳ yenile" butonuna basıldığında `ros2 run tf2_tools view_frames` komutunu arka planda çalıştırır ve **o an sisteminizde yayınlanan TF frame'lerini** otomatik olarak çeker. Statik hiçbir veri yoktur — her bilgisayarda kendi TF ağacını gösterir.

```
Tarayıcı (React) ←→ Node.js backend (server.cjs) → ros2 view_frames
```

## Özellikler

- O an çalışan ROS2 sisteminden canlı TF verisi çeker
- `sim_time` toggle'ı ile simülasyon zamanı desteği
- Her derinlik seviyesi farklı renkte gösterilir
- Düğümlere tıklayarak alt ağacı daraltıp açabilirsiniz
- Terminal tree formatında panoya kopyalama
- Frame sayısı, derinlik ve son güncelleme saati

---

## Kurulum (Ubuntu 22.04)

### 1. Node.js Kurulumu (nvm ile)

```bash
# nvm kur
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash

# Terminali yenile
source ~/.bashrc

# Node.js 20 kur
nvm install 20
nvm use 20

# Doğrula
node --version   # v20.x.x çıkmalı
npm --version    # 10.x.x çıkmalı
```

### 2. Projeyi İndir

```bash
git clone https://github.com/KULLANICI_ADIN/tf-tree.git
cd tf-tree
```

### 3. Bağımlılıkları Yükle

```bash
npm install
```

---

## Çalıştırma

Her seferinde **iki terminal** açmanız gerekir:

**Terminal 1 — Backend:**
```bash
cd tf-tree
node server.cjs
```
Şu çıktıyı görmelisiniz:
```
✓ TF Tree backend çalışıyor → http://localhost:8000
```

**Terminal 2 — Frontend:**
```bash
cd tf-tree
npm run dev
```

Tarayıcıda aç: [http://localhost:5173](http://localhost:5173)

Durdurmak için her iki terminalde: `Ctrl + C`

---

## Kullanım

1. Uygulama açıldığında otomatik olarak `ros2 run tf2_tools view_frames` çalıştırır (~5 saniye sürer)
2. TF ağacı ekrana yansır
3. **"⟳ yenile"** butonuyla istediğiniz zaman güncel veriyi çekebilirsiniz
4. Simülasyon kullanıyorsanız **sim_time** toggle'ını açın

---

## Gereksinimler

| Araç | Versiyon |
|------|----------|
| Node.js | v20+ |
| npm | v10+ |
| ROS2 | Humble / Iron / Jazzy |
| tf2_tools | `sudo apt install ros-$ROS_DISTRO-tf2-tools` |
| İşletim Sistemi | Ubuntu 22.04 |

---

## Sorun Giderme

**"Backend'e bağlanılamadı" hatası:**
```bash
# Backend çalışıyor mu?
curl http://localhost:8000/health

# Çalışmıyorsa başlat:
cd tf-tree && node server.cjs
```

**TF verisi gelmiyor:**
```bash
# ROS2 çalışıyor mu?
ros2 topic list

# tf2_tools kurulu mu?
sudo apt install ros-$ROS_DISTRO-tf2-tools
```
