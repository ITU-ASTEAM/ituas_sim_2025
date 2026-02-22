import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray # Gazebo'daki Pose_V verisi için
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from ultralytics import YOLO

class MistikGazeboIntegratedNode(Node):
    def __init__(self):
        super().__init__('mistik_vision_node')
        self.bridge = CvBridge()
        
        # YOLOv8/10 Modelini yükle (HP Victus GPU desteğiyle)
        try:
            self.model = YOLO('best29jan.pt') 
            self.get_logger().info("✅ YOLO Modeli ve GPU aktif.")
        except Exception as e:
            self.get_logger().error(f"❌ Model dosyası (last.pt) bulunamadı: {e}")

        self.latest_depth_frame = None
        self.ship_x, self.ship_y, self.ship_yaw = 0.0, 0.0, 0.0
        
        # --- Duba Hafızası ---
        self.known_buoys = []  
        self.next_id = 1
        self.dist_threshold = 1.0 # Gazebo için 1 metrelik mesafe eşiği idealdir

        # Gazebo Humble için Best Effort QoS ayarı
        gazebo_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --- ABONELİKLER (Topic isimleri senin Gazebo listene göre güncellendi) ---
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, gazebo_qos)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, gazebo_qos)
        self.create_subscription(PoseArray, '/world/ocean_world/pose/info', self.pose_callback, gazebo_qos)

        # Yayıncılar
        self.pub_pc = self.create_publisher(PointCloud2, '/mistik_points', 10)
        self.pub_img = self.create_publisher(Image, '/mistik_detection_image', 10)
        
        # Kamera Sabitleri (Gazebo Varsayılan)
        self.base_f_l = 440.4 
        self.base_width = 640.0

        self.get_logger().info("🚀 Gazebo Entegrasyonu Tamam! Veriler terminale akıyor...")

    def pose_callback(self, msg):
        """Gazebo'daki teknenin anlık koordinatlarını (x, y, yaw) çeker."""
        if len(msg.poses) > 0:
            target_pose = msg.poses[0] #
            self.ship_x = target_pose.position.x
            self.ship_y = target_pose.position.y
            
            # Quaternion -> Yaw (Açı) Hesabı
            q = target_pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.ship_yaw = math.atan2(siny_cosp, cosy_cosp)

    def depth_callback(self, msg):
        """Gazebo metre cinsinden float32 (32FC1) derinlik basar."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_frame = np.array(cv_image, dtype=np.float32)
        except: pass

    def get_global_id(self, label, distance, angle_rad):
        """Dubanın dünyadaki X, Y koordinatını hesaplar ve ID atar."""
        total_angle = self.ship_yaw + angle_rad
        # Global X, Y Hesaplama Formülü:
        # $X_{buoy} = X_{ship} + (d \cdot \cos(\theta_{total}))$
        # $Y_{buoy} = Y_{ship} + (d \cdot \sin(\theta_{total}))$
        buoy_gx = self.ship_x + (distance * math.cos(total_angle))
        buoy_gy = self.ship_y + (distance * math.sin(total_angle))

        assigned_id = None
        min_dist = float('inf')

        for buoy in self.known_buoys:
            if buoy['label'] == label:
                dist_to_buoy = math.sqrt((buoy['x'] - buoy_gx)**2 + (buoy['y'] - buoy_gy)**2)
                if dist_to_buoy < self.dist_threshold:
                    if dist_to_buoy < min_dist:
                        min_dist = dist_to_buoy
                        assigned_id = buoy['id']
                        # Hareketli ortalama ile konumu yumuşat
                        buoy['x'] = buoy['x'] * 0.9 + buoy_gx * 0.1
                        buoy['y'] = buoy['y'] * 0.9 + buoy_gy * 0.1
        
        if assigned_id is None:
            assigned_id = self.next_id
            self.known_buoys.append({'id': assigned_id, 'x': buoy_gx, 'y': buoy_gy, 'label': label})
            self.next_id += 1
            self.get_logger().info(f"🆕 HARİTAYA EKLENDİ: ID {assigned_id} - {label}")

        return assigned_id, buoy_gx, buoy_gy

    def rgb_callback(self, msg):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except: return

        if self.latest_depth_frame is None: return

        rgb_h, rgb_w = cv_rgb.shape[:2]
        depth_h, depth_w = self.latest_depth_frame.shape[:2]
        current_f_l = self.base_f_l * (rgb_w / self.base_width)
        scale_x, scale_y = depth_w / rgb_w, depth_h / rgb_h

        # YOLOv8 çıkarımı
        results = self.model.predict(cv_rgb, conf=0.4, verbose=False, device=0)
        point_cloud_data = []

        for r in results[0].boxes:
            x1, y1, x2, y2 = map(int, r.xyxy[0])
            label = self.model.names[int(r.cls[0])]
            u_c, v_c = (x1 + x2) // 2, (y1 + y2) // 2

            angle_rad = math.atan((u_c - (rgb_w / 2)) / current_f_l)
            angle_deg = math.degrees(angle_rad)
            
            # Derinlik bilgisini ROI (İlgi Alanı) üzerinden alıyoruz
            u_d, v_d = int(u_c * scale_x), int(v_c * scale_y)
            roi = self.latest_depth_frame[max(0,v_d-5):min(depth_h,v_d+5), max(0,u_d-5):min(depth_w,u_d+5)]
            valid_pixels = roi[~np.isnan(roi) & (roi > 0.1) & (roi < 35.0)]

            if len(valid_pixels) > 0:
                dist_m = np.nanmin(valid_pixels)
                buoy_id, gx, gy = self.get_global_id(label, dist_m, angle_rad)
                
                # --- TERMİNAL AKIŞI ---
                print(f"TESPİT: {label:10} | ID: {buoy_id:2} | Mesafe: {dist_m:.2f}m | Açı: {angle_deg:5.1f}° | X: {gx:.2f} Y: {gy:.2f}", flush=True)
                
                # Görselleştirme
                cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_rgb, f"ID:{buoy_id} {dist_m:.1f}m", (x1, y1-10), 0, 0.5, (0, 255, 0), 2)
                
                # RViz için PointCloud Verisi
                x_m = dist_m * math.tan(angle_rad)
                point_cloud_data.append([dist_m, -x_m, 0.0])

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(cv_rgb, 'bgr8'))
        cv2.imshow("Mistik Gazebo Live", cv_rgb)
        cv2.waitKey(1)

        if point_cloud_data:
            h = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_link")
            self.pub_pc.publish(pc2.create_cloud_xyz32(h, point_cloud_data))

def main():
    rclpy.init()
    node = MistikGazeboIntegratedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
