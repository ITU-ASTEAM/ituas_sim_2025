# gazebo
Merhabalar:
setup.py ile kurulum aşamaları:
1) Bilgisayarınızda Git ile SSH bağlantısı olduğuna emin olun.
2) my_robot_ws dosyasının masaüstünde olduğuna emin olunuz ayrıca src/ içindeki vrx klasörünü siliniz. (vrx dosyalarını githubdan çekerken klasör var diye çekmez yoksa)



final_boat_hydro.sdf: Ortam ve fiziklerin olduğu ana dünya dosyası.
model.sdf: Teknenin kendi model dosyası.

Build aşaması: (.sdf dosyalarındaki dosya yollarından emin olun)
colcon build
source install/setup.bash

simülasyonu başlatmak için
gz sim -v 4 -r Desktop/my_robot_ws/src/my_robot_description/worlds/final_boat_hydro.sdf

Gemiyi hareket ettirmek için kontrol.py scriptini çalıştır. Terminalde açık kalsın, WASD ile sürüyorsun:
python3 Scripts/kontrol.py


Kamera verisi için (RGB ve Derinlik)
python3 scripts/goruntuisleme/calistir2.py


Üst üste gazebolar çakışırsa
pkill -9 -f ign && pkill -9 -f gazebo && pkill -9 -f ruby

daha kökten bir çözüm
sudo pkill -9 -f 'ign|gazebo|ruby|gz' && sudo rm -rf /dev/shm/ign-* /dev/shm/gz-* /tmp/.ignition /tmp/.gz && echo "Temizlik tamamlandı!"


İlerde scriptlerin işlevleri daha detaylı anlatılacaktır.


Golet.dae githubın 100 mb limitini aştığı için drive linki ekledim isteyen indirebilir.
https://drive.google.com/drive/folders/1548BU8YdyO9oBsV8xFsSzGwSMaZHzrnU?usp=sharing


https://drive.google.com/drive/folders/1XHSol1CxVLURtLInUD2BIfSHCaRn7t5O?usp=drive_link
simülasyon görüntü işleme eğitim datası

