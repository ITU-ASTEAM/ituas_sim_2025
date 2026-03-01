import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import threading
import time

class ScanMatcher(Node):
    def __init__(self):
        super().__init__('scan_matcher_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback,
            10)
        self.capture_flag = False
        self.current_scan = None
        self.merged_cloud = o3d.geometry.PointCloud()
        self.scan_count = 0
        self.max_scans = 10

    def listener_callback(self, msg):
        if self.capture_flag:
            # ROS 2 PointCloud2 verisini x,y,z formatında Numpy dizisine çevir
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(gen))
            
            # Numpy dizisini Open3D nokta bulutu formatına dönüştür
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            self.current_scan = pcd
            self.capture_flag = False # Veri alındı, bayrağı indir

    def capture_and_process(self):
        print("\n--- 3D LiDAR Manuel Tarama ve Eşleştirme Başladı ---")
        print("Uyarı: Taramalar arası hareketi çok abartmayın, ortak yüzeyler görünmeli.\n")

        for i in range(self.max_scans):
            input(f"Adım {i+1}/{self.max_scans}: Cihazı konumlandırın ve tarama almak için ENTER'a basın...")
            
            self.current_scan = None
            self.capture_flag = True
            
            # Callback'in veriyi yakalamasını bekle
            while self.current_scan is None and rclpy.ok():
                time.sleep(0.1)
            
            print(f"[{i+1}] Tarama alındı. ICP ile eşleştiriliyor...")
            
            # Veriyi işleme hızını ve kalitesini artırmak için downsampling (nokta seyreltme)
            current_down = self.current_scan.voxel_down_sample(voxel_size=0.05)

            if self.scan_count == 0:
                # İlk tarama haritanın merkezidir
                self.merged_cloud = current_down
            else:
                # ICP Eşleştirme (Maksimum 2 metre uzaklıktaki noktaları dikkate alır)
                threshold = 2.0 
                trans_init = np.identity(4) # Başlangıç tahmini yok (Birim matris)
                
                reg_p2p = o3d.pipelines.registration.registration_icp(
                    current_down, self.merged_cloud, threshold, trans_init,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
                
                # Yeni taramayı hesaplanan matris ile kaydırıp ana haritaya ekle
                current_down.transform(reg_p2p.transformation)
                self.merged_cloud += current_down
                
                # Üst üste binen noktaları temizlemek için tekrar downsample yap
                self.merged_cloud = self.merged_cloud.voxel_down_sample(voxel_size=0.03)

            self.scan_count += 1
            print(f"[{self.scan_count}] Haritaya eklendi. Toplam nokta sayısı: {len(self.merged_cloud.points)}\n")

        # Sonucu diske kaydet
        output_filename = "vlp16_merged_room.pcd"
        o3d.io.write_point_cloud(output_filename, self.merged_cloud)
        print(f"İşlem tamam. {self.max_scans} tarama birleştirildi.")
        print(f"Çıktı '{output_filename}' olarak kaydedildi.")

def main(args=None):
    rclpy.init(args=args)
    node = ScanMatcher()
    
    # input() fonksiyonu ROS callback'lerini bloklamasın diye Spin işlemini ayrı thread'e alıyoruz
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        node.capture_and_process()
    except KeyboardInterrupt:
        print("\nİşlem kullanıcı tarafından iptal edildi.")
    finally:
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
