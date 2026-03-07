import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from ultralytics import YOLO

class MistikFinalNode(Node):
    def __init__(self):
        super().__init__('mistik_vision_node')
        self.bridge = CvBridge()
        self.model = YOLO('best.pt') 
        self.latest_depth_frame = None
        
        # Abonelikler (Görüntü ve Derinlik)
        self.create_subscription(Image, '/oak/rgb/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/oak/stereo/image_raw', self.depth_callback, 10)

        # Yayıncılar
        self.pub_pc = self.create_publisher(PointCloud2, '/mistik_points', 10)
        self.pub_img = self.create_publisher(Image, '/mistik_detection_image', 10)
        
        # Kamera Sabitleri (OAK-D Specs)
        self.base_f_l = 440.4 
        self.base_width = 640.0

        self.get_logger().info("Mistik Log Sistemi Aktif: Mesafe ve Açı terminale basılıyor.")

    def depth_callback(self, msg):
        try:
            # Derinlik verisini mm cinsinden al
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_frame = np.array(cv_image, dtype=np.uint16)
        except Exception as e:
            self.get_logger().error(f"Depth hatası: {e}")

    def rgb_callback(self, msg):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception: return

        if self.latest_depth_frame is None: return

        rgb_h, rgb_w = cv_rgb.shape[:2]
        depth_h, depth_w = self.latest_depth_frame.shape[:2]
        current_f_l = self.base_f_l * (rgb_w / self.base_width)
        scale_x, scale_y = depth_w / rgb_w, depth_h / rgb_h

        results = self.model.predict(cv_rgb, conf=0.25, verbose=False)
        point_cloud_data = []

        for r in results[0].boxes:
            x1, y1, x2, y2 = map(int, r.xyxy[0])
            label = self.model.names[int(r.cls[0])]
            u_c, v_c = (x1 + x2) // 2, (y1 + y2) // 2

            # Açı Hesabı
            pixel_offset = u_c - (rgb_w / 2)
            angle_rad = math.atan(pixel_offset / current_f_l)
            angle_deg = math.degrees(angle_rad)

            # Mesafe Hesabı (Hizalama hataları için np.min)
            u_d, v_d = int(u_c * scale_x), int(v_c * scale_y)
            roi = self.latest_depth_frame[max(0,v_d-10):min(depth_h,v_d+10), max(0,u_d-10):min(depth_w,u_d+10)]
            valid_pixels = roi[(roi > 100) & (roi < 10000)]

            if len(valid_pixels) > 0:
                z = np.min(valid_pixels) / 1000.0 # Metreye çevir
                
                # TERMINAL LOGLAMA (İstediğin kısım burası kral)
                self.get_logger().info(f"TESPİT: {label} | Mesafe: {z:.2f}m | Açı: {angle_deg:.1f}°")

                # Görselleştirme (RViz Ekranı İçin)
                info_text = f"{z:.2f}m | {angle_deg:.1f}deg"
                cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_rgb, info_text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 3D PointCloud verisi hazırlama
                x_m = z * math.tan(angle_rad)
                y_m = (v_c - (rgb_h/2)) * z / current_f_l
                point_cloud_data.append([z, -x_m, -y_m])

        # Yayınlama
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(cv_rgb, 'bgr8'))
        if point_cloud_data:
            h = Header(stamp=self.get_clock().now().to_msg(), frame_id="oak_rgb_camera_optical_frame")
            self.pub_pc.publish(pc2.create_cloud_xyz32(h, point_cloud_data))

def main():
    rclpy.init(); rclpy.spin(MistikFinalNode()); rclpy.shutdown()

if __name__ == '__main__':
    main()
