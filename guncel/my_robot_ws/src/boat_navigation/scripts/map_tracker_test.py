#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import cv2
import os

class MapPositionTester(Node):
    def __init__(self):
        super().__init__('map_position_tester')

        # TF2 Tanımlamaları
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Harita Parametreleri (costmap.yaml içindeki kesin değerlerini buraya gir)
        self.resolution = 0.05
        self.origin_x = -64.2
        self.origin_y = -25.5

        # Dosya Yolları (gauss bilgisayarına göre ayarlandı)
        self.map_path = '/home/gauss/Desktop/my_robot_ws/src/boat_navigation/maps/costmap.png'
        self.save_path = '/home/gauss/Desktop/my_robot_ws/src/boat_navigation/maps/costmap_tracked.png'

        # Haritayı belleğe yükle
        if os.path.exists(self.map_path):
            self.map_image = cv2.imread(self.map_path)
            self.img_height, self.img_width, _ = self.map_image.shape
            self.get_logger().info('Statik harita yüklendi. Konum takibi başlıyor...')
        else:
            self.get_logger().error(f'Harita bulunamadı: {self.map_path}')
            return

        # Saniyede 2 kez konumu alıp çizecek döngü
        self.timer = self.create_timer(0.5, self.track_and_draw)

    def track_and_draw(self):
        try:
            # map'ten base_link'e olan anlık uzaysal konumu oku
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y

            # Metreyi Piksele Çevirme (ROS -> OpenCV koordinat matematiği)
            # ROS'ta orijin sol alt, OpenCV'de sol üst köşedir. Y ekseni tersine çevrilir.
            px = int((x - self.origin_x) / self.resolution)
            py = self.img_height - int((y - self.origin_y) / self.resolution)

            # Sınır kontrolü
            if 0 <= px < self.img_width and 0 <= py < self.img_height:
                # BGR formatında saf kırmızı nokta çiz
                cv2.circle(self.map_image, (px, py), radius=3, color=(0, 0, 255), thickness=-1)
                
                # Resmi üstüne yazarak kaydet
                cv2.imwrite(self.save_path, self.map_image)
                self.get_logger().info(f'X:{x:.1f}, Y:{y:.1f} -> Piksel:({px}, {py}) kaydedildi.')
            else:
                self.get_logger().warning('Gemi harita sınırlarının dışında!')

        except Exception as e:
            # TF ağacı henüz hazır değilse bekle
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MapPositionTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
