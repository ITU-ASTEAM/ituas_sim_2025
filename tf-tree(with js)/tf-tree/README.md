# 🌳 ROS2 TF Tree Viewer

Terminal `tree` komutu gibi ROS2 TF frame ağacını görselleştiren web uygulaması.

![Demo](/ornek.png)

## Özellikler

- `├──` / `└──` formatındaki TF ağaçlarını görselleştirir
- Girintili liste formatını da destekler
- Her derinlik seviyesi farklı renkte gösterilir
- Düğümlere tıklayarak alt ağacı daraltıp açabilirsiniz
- Terminal tree formatında panoya kopyalama
- Frame sayısı ve maksimum derinlik bilgisi

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
### 2. Dosyaları bilgisayara indirin

```

### 3. Bağımlılıkları Yükle

```bash
npm install
```

---

## Çalıştırma

```bash
cd tf-tree
npm run dev
```

Tarayıcıda aç: [http://localhost:5173](http://localhost:5173)

Durdurmak için: `Ctrl + C`

---

## Kullanım

Uygulamayı açtıktan sonra:

1. **"düzenle"** butonuna tıklayın
2. TF ağacınızı yapıştırın (iki format desteklenir):

**Format 1 — Girintili liste:**
```
world
  map
    odom
      base_link
        laser_link
        camera_link
```

**Format 2 — Tree formatı (`ros2 run tf2_tools view_frames` çıktısı):**
```
world
└── map
    └── odom
        └── base_link
            ├── laser_link
            └── camera_link
```

3. **"Görselleştir →"** butonuna tıklayın

---

## Gereksinimler

| Araç | Versiyon |
|------|----------|
| Node.js | v20+ |
| npm | v10+ |
| İşletim Sistemi | Ubuntu 22.04 (veya herhangi bir Linux/macOS/Windows) |

---
