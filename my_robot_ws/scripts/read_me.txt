Bu not, daha önce oluşturulmuş ancak henüz çalıştırılmamış Gazebo simülasyonlarının nasıl başlatılacağını, simülasyonun adım adım nasıl açıldığını ve teknenin klavye üzerinden nasıl kontrol edilebildiğini açıklamak amacıyla hazırlanmıştır.

Gazebo ortamında oluşturulan simülasyon, final_boat.sdf isimli bir SDF (Simulation Description Format) dosyası ile tanımlanmıştır. İlgili klasör yapısı Ubuntu işletim sistemi üzerinde Downloads dizini altına yerleştirildiğinde, simülasyon terminal üzerinden çalıştırılabilmektedir. Simülasyonu başlatmak için gz " sim ~/Downloads/my_robot_ws/install/my_robot_description/share/my_robot_description/worlds/final_boat.sdf " komutu kullanılmakta olup, bu komut ile birlikte tekne modeli ve dünya ortamı Gazebo Sim içerisinde yüklenerek çalıştırılmaktadır.

Simülasyon ortamı doğrudan klavye girdilerini algılamadığı için, klavye üzerinden verilen komutların teknenin motorlarına veya iticilerine aktarılabilmesi amacıyla boat_keyboard.py isimli bir Python kontrol kodu kullanılmaktadır. Bu dosya içerisinde W, A, S ve D tuşları tanımlanmış olup, klavye üzerinden manuel kontrol sağlanmaktadır.

Her tuş komutu tek seferlik bir ivme etkisi oluşturacak şekilde tasarlanmıştır. Ters yöndeki tuşa bir kez basıldığında ise bir önceki komut sıfırlanmaktadır. Örneğin, W tuşuna bir kez basıldığında gemi ileri yönde hareket etmeye başlar; ardından S tuşuna bir kez basıldığında gemi durur ve S tuşuna tekrar basıldığında geri yönde hareket eder.

Her komut için belirli bir bekleme süresi tanımlanmıştır. Simülasyon deniz ortamını temsil ettiğinden dolayı, teknenin hareketlerinde suyun fiziksel etkisini yansıtan belirli bir kayma ve savrulma davranışı gözlemlenmektedir. Bu durum, hareketlerin anlık olarak kesilmemesine ve daha gerçekçi bir dinamik yapının oluşmasına katkı sağlamaktadır.

Terminal üzerinden klavye kontrol kodunu çalıştırabilmek için öncelikle dosyanın bulunduğu dizine geçilmesi gerekmektedir. Bunun için terminalde " cd ~/Downloads/my_robot_ws/scripts " komutu çalıştırılır. Ardından boat_keyboard.py dosyasını çalıştırmak için " ./boat_keyboard.py " komutu kullanılır ve klavye kontrolü aktif hâle getirilir.

Gazebo arayüzünde simülasyon başlatma butonuna tıklandıktan sonra simülasyon çalışmaya başlar. Klavye komutlarının algılanabilmesi için terminal penceresine tıklanarak terminalin aktif hâle getirilmesi gerekmektedir; aksi takdirde tuş girişleri algılanmaz.

Bundan sonraki aşamalarda yapılacak tüm çalışmalar, simülasyonun fiziksel gerçekçiliğini artırmak, motor ve hidrodinamik parametreleri iyileştirmek, kontrol yapısını daha kararlı hâle getirmek ve ilerleyen süreçte otonom kontrol algoritmalarının eklenebilmesi için gerekli altyapıyı oluşturmak amacıyla gerçekleştirilecektir. Yapılan her düzenlemenin hangi problem için yapıldığı ve simülasyona olan etkisi ayrıntılı olarak açıklanacaktır.

cd ~/Downloads/my_robot_ws/scripts
chmod +x boat_mission.py
python3 boat_mission.py


gz sim /home/beequeen/Downloads/my_robot_ws/install/my_robot_description/share/my_robot_description/worlds/final_boat.sdf
gazeboda final_boat açıyor 



<!-- GOLET PARKUR 
<include>
<uri>file:///home/beequeen/Downloads/my_robot_ws/src/my_robot_description/models/itu_lake</uri>
<name>itu_lake</name>
<pose>0 0 0 0 0 0</pose>
</include>

-->
