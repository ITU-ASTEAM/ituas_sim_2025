\# Pixhawk Telemetry Dashboard



Gerçek zamanlı MAVLink telemetri verilerini izlemek ve araç kontrolü sağlamak için geliştirilmiş, sürükle-bırak destekli PyQt5 tabanlı bir arayüz.



\## Kurulum



Gerekli kütüphaneleri yükleyin:



```bash

pip install pymavlink pyqtgraph PyQt5 PyQtWebEngine

```



\## Mission Planner Ayarı



Mission Planner'da aşağıdaki ayarları yapın:



\- \*\*Ctrl + F → Mavlink\*\*

\- Type: `UDP`

\- Direction: `Outbound`

\- Host: `127.0.0.1`

\- Port: `14550`

\- \*\*Write\*\* seçili olmalı.

\- \*\*Go\*\* butonuna basın.



\## Çalıştırma



```bash

python mavlink\_live\_plot\_v2.py

```



\## Temel Özellikler



\- Gerçek zamanlı MAVLink telemetri görüntüleme

\- Sürükle-bırak destekli özelleştirilebilir dashboard

\- Hız, heading, pitch, roll ve thruster verilerini izleme

\- Canlı GPS haritası

\- Pusula ve yapay ufuk göstergeleri

\- ARM / DISARM ve araç modu değiştirme

\- Dashboard düzenini kaydetme ve otomatik yükleme

\- Python scriptlerini arayüz üzerinden çalıştırabilme (DoAction)

