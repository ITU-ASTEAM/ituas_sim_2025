# VeloView

**VeloView**, Velodyne lidarların verisini gerçek zamanlı görmek, kaydetmek ve yeniden oynatmak için kullanılan açık kaynak bir masaüstü uygulamasıdır. Kitware tarafından ParaView üzerine geliştirilmiştir; proje bugün **LidarView** adıyla sürdürülüyor.

Bu klasör, **VeloView 5.1.0**'ın Ubuntu 18.04 / x86_64 için derlenmiş hazır paketidir. Kurulum gerektirmez: gereken kütüphaneler (Qt, ParaView 5.9, Python 3.9, numpy) paketin içinde gelir.

## Ne işe yarar

- **Lidarı ROS'tan bağımsız test etmek:** VLP-16 açılıyor mu, ağ bağlantısı doğru mu, nokta bulutunda montaj ya da kablo kaynaklı bir sorun var mı; ROS tarafına geçmeden önce buradan görülür.
- **Kayıt almak:** Lidarın ham paketleri `.pcap` dosyası olarak kaydedilir; saha testinden sonra aynı veri tekrar tekrar incelenebilir.
- **Kaydı oynatmak:** `.pcap` dosyası VeloView'da açılıp kare kare izlenebilir. `PacketFileSender` ise kaydı ağ üzerinden göndererek gerçek bir lidar bağlıymış gibi davranır; böylece lidar olmadan sürücü ve algılama kodu denenebilir.

Lidarı ROS içinde görmek için `velodyne` sürücüsü ve [`tools/rviz/velocity.rviz`](../rviz/README.md) kullanılır.

## Çalıştırma

```bash
cd tools/veloview
./bin/VeloView
```

Uygulama açıldıktan sonra **Open Sensor Stream** ile canlı veriyi açın ve sensör olarak **VLP-16**'yı seçin. Kayıtlı bir dosya için **Open** ile `.pcap` dosyasını açın.

## Ağ ayarları

| Ayar | Varsayılan değer |
|---|---|
| Lidar IP adresi | `192.168.1.201` |
| Veri portu (UDP) | `2368` |
| Konum/GPS portu (UDP) | `8308` |

- Bilgisayarın kablolu ağ arayüzüne aynı alt ağdan sabit bir IP verilmelidir (örneğin `192.168.1.100/24`).
- Bağlantıyı kontrol etmek için: `ping 192.168.1.201`
- Lidarın ayar arayüzü: `http://192.168.1.201/` (https değil)

Varsayılan değerler lidarın ayar arayüzünden değiştirilmiş olabilir; emin olmak için önce ayar arayüzüne bakın.

## Klasör yapısı

| Yol | İçerik |
|---|---|
| `bin/VeloView` | Uygulamanın kendisi |
| `bin/PacketFileSender` | `.pcap` kaydını ağ üzerinden gönderen araç |
| `bin/qt.conf`, `bin/VeloView.conf` | Göreli yol ayarları; bu sayede klasör başka bir yere taşınabilir |
| `lib/` | Paketle gelen kütüphaneler (Qt, ParaView/VTK 5.9, Python 3.9, numpy) |
| `plugins/`, `share/` | Eklentiler ve uygulama kaynakları |

## Notlar

- Paket Ubuntu 18.04 için derlenmiştir. Ubuntu 24.04 üzerinde çalıştığı bu repoda doğrulanmadı.
- VeloView'un kaynak kodu Apache 2.0 lisanslıdır. Proje sayfası: [Kitware Wiki: VeloView](https://public.kitware.com/Wiki/VeloView). Güncel sürümler Kitware'in GitLab'ında LidarView adıyla yayınlanıyor.
