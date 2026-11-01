#!/usr/bin/env python3
"""
Njord Füzyon Sistemi v8.0 — Kamera+Derinlik Görü Node'u (YOLO)
- Bu process SADECE YOLO inference + derinlik ROI hesabıyla ilgilenir. Duba ID/pozisyon
  otoritesi burada DEĞİLDİR — sadece bir (x,y,renk_etiketi) adayı üretip
  `/vision/buoy_detections` topic'ine yayınlar; asıl LiDAR track'ine bağlama ve renk
  oylaması `lidar_fusion_node.py` tarafında yapılır (bkz. o dosyadaki
  `buoy_detection_callback`/`_fuse_camera_detection`).
- `lidar_fusion_node.py`'den AYRI bir OS process'i olarak `calistir.py` ile başlatılır.
  Amaç: YOLO inference'ının (Python/GIL ağırlıklı, Jetson'da CPU/GPU yoğun) LiDAR
  callback'ini asla bloklamaması ve Jetson Orin NX'in çekirdeklerinin iki gerçek process
  arasında paylaşılması (threading'in GIL sınırlamasının aksine).
- Gemi pozu (ship_x, ship_y, fused_yaw) artık paylaşılan Python state değil,
  `/njord/ego_pose` topic'inden okunur. Mapping açık/kapalı durumu da
  `/njord/mapping_active` topic'inden okunur.
- TensorRT: Jetson Orin NX'te `best1JUL.engine` (TensorRT engine, export adımları
  README/plan notlarında) varsa öncelikli yüklenir; yoksa `.pt` ile (yavaş) devam
  edilir ve bir uyarı loglanır.
"""

from pathlib import Path
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy,
                        QoSReliabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

from oak_common import EGO_POSE_STALE_WARN_S


class NjordCameraVisionNode(Node):
    def __init__(self):
        super().__init__('njord_camera_vision_node')

        self.get_logger().info("🟢 ROS 2 Kamera/Görü Node'u Başlatılıyor...")
        self.bridge = CvBridge()

        # __file__'a göre çözülür (cwd'ye göre DEĞİL) — script başka bir dizinden
        # başlatılsa bile (örn. calistir.py olmadan doğrudan) yanındaki engine'i bulur.
        engine_path = Path(__file__).resolve().parent / 'best1JUL.engine'
        if engine_path.exists():
            model_path = str(engine_path)
            self.get_logger().info(f"🧠 TensorRT engine yükleniyor: {model_path}")
        else:
            model_path = str(Path(__file__).resolve().parent / 'best1JUL.pt')
            self.get_logger().warn(
                "⚠️  'best1JUL.engine' bulunamadı — '.pt' ile (yavaş, TensorRT'siz) "
                "çalışılıyor. Jetson Orin NX'te 'yolo export model=best1JUL.pt "
                "format=engine device=0 half=True' ile engine üretip yanına koyun.")
        # task='segment' AÇIKÇA belirtiliyor: .pt dosyası SegmentationModel (maske
        # başlı) olarak eğitilmiş, ama TensorRT .engine dosyaları bu görev bilgisini
        # SAKLAMIYOR — ultralytics engine'i yüklerken bunu tahmin edemeyip sessizce
        # 'detect' varsayar, bu da çıktı tensöründeki maske katsayılarının kutu/sınıf
        # ayrıştırmasına karışıp YANLIŞ tespitler üretmesine yol açabilir. Açıkça
        # belirtmek bu belirsizliği tamamen ortadan kaldırır.
        self.model = YOLO(model_path, task='segment')
        self.get_logger().info("✅ YOLO Modeli hazır!")

        # --- TEŞHİS BAYRAKLARI ---
        self.status = {'rgb': False, 'depth': False}
        self._last_yolo_infer_ms = 0.0

        # --- LiDAR NODE'DAN GELEN PAYLAŞILAN DURUM (topic üzerinden) ---
        self.mapping_active = False
        self.ship_x, self.ship_y, self.fused_yaw = 0.0, 0.0, 0.0
        # /njord/ego_pose ne zaman geldi — LiDAR node yavaşlar/düşerse bu node'un ELİNDEKİ
        # pozun BAYATLADIĞINI fark edip loglayabilmek için (bkz. rgb_callback, kullanıcı
        # geri bildirimi: en azından loglanmalı, sessizce yanlış konum üretilmemeli).
        self._last_ego_pose_wall_time = None

        self.base_f_l = 440.4
        self.base_width = 640.0
        self.image_width = 0
        self.current_f_l = self.base_f_l
        self.latest_depth_frame = None

        # --- YAYINCILAR ---
        self.buoy_detections_pub = self.create_publisher(String, '/vision/buoy_detections', 10)
        self.yolo_img_pub = self.create_publisher(Image, '/vision/yolo_output', 10)

        # --- CALLBACK GRUPLARI ---
        # rgb/YOLO (ağır) kendi grubunda; derinlik + gemi pozu + mapping durumu (hafif)
        # ayrı bir grupta — YOLO inference sürerken derinlik karesi/poz güncellemesi
        # gecikmeden işlenmeye devam eder.
        self.cb_fast = MutuallyExclusiveCallbackGroup()
        self.cb_camera = MutuallyExclusiveCallbackGroup()

        mapping_state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        sensor_qos = qos_profile_sensor_data
        self.create_subscription(Image, '/oak/stereo/image_raw', self.depth_callback,
                                  sensor_qos, callback_group=self.cb_fast)
        self.create_subscription(Odometry, '/njord/ego_pose', self.ego_pose_callback,
                                  10, callback_group=self.cb_fast)
        self.create_subscription(Bool, '/njord/mapping_active', self.mapping_active_callback,
                                  mapping_state_qos, callback_group=self.cb_fast)
        self.create_subscription(Image, '/oak/rgb/image_raw', self.rgb_callback,
                                  sensor_qos, callback_group=self.cb_camera)

        self.create_timer(2.0, self.print_diagnostic_status, callback_group=self.cb_fast)

    # -----------------------------------------------------------------------
    # TEŞHİS
    # -----------------------------------------------------------------------
    def print_diagnostic_status(self):
        pose_age = self._ego_pose_age_s()
        pose_age_str = "veri yok" if pose_age is None else f"{pose_age*1000:.0f} ms önce"
        status_msg = (
            f"\n--- KAMERA/GÖRÜ NODE DURUMU ---\n"
            f"📷 RGB Kamera     : {'✅ AKTİF' if self.status['rgb'] else '❌ BEKLENİYOR'} "
            f"(YOLO inference: {self._last_yolo_infer_ms:.0f} ms)\n"
            f"📏 Depth Kamera   : {'✅ AKTİF' if self.status['depth'] else '❌ BEKLENİYOR'}\n"
            f"🧭 Gemi Pozu (lidar node'dan): son alınma {pose_age_str}\n"
            f"Mapping (lidar node'dan): {'AKTIF' if self.mapping_active else 'KAPALI'}\n"
            f"-------------------"
        )
        self.get_logger().info(status_msg)
        for key in self.status:
            self.status[key] = False

    # -----------------------------------------------------------------------
    # LiDAR NODE'DAN GELEN DURUM GÜNCELLEMELERİ
    # -----------------------------------------------------------------------
    def ego_pose_callback(self, msg: Odometry):
        self.ship_x = msg.pose.pose.position.x
        self.ship_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.fused_yaw = 2.0 * math.atan2(q.z, q.w)
        self._last_ego_pose_wall_time = time.time()

    def _ego_pose_age_s(self):
        if self._last_ego_pose_wall_time is None:
            return None
        return time.time() - self._last_ego_pose_wall_time

    def mapping_active_callback(self, msg: Bool):
        self.mapping_active = msg.data

    # -----------------------------------------------------------------------
    # GÖRÜNTÜ İŞLEME (KAMERA → RENK ADAYI ÜRETİMİ)
    # -----------------------------------------------------------------------
    def depth_callback(self, msg):
        self.status['depth'] = True
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_frame = np.array(cv_img, dtype=np.uint16)
        except Exception: pass

    def rgb_callback(self, image_msg: Image):
        self.status['rgb'] = True
        if not self.mapping_active:
            return  # '1'e basılmadan YOLO/füzyon çalışmaz
        try: cv_rgb = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except Exception: return

        # Gemi pozu bayatsa (LiDAR node yavaşlamış/düşmüş olabilir) bu karedeki duba
        # konumları eski poza dayanacak — tespiti yine de üretiyoruz ama kullanıcı
        # geri bildirimi gereği en azından açıkça logluyoruz.
        pose_age = self._ego_pose_age_s()
        if pose_age is None or pose_age > EGO_POSE_STALE_WARN_S:
            self.get_logger().warn(
                f"[POZ] /njord/ego_pose bayat veya hiç gelmedi (yaş="
                f"{'veri yok' if pose_age is None else f'{pose_age*1000:.0f}ms'}) — "
                f"bu karedeki duba konumları eski/varsayılan gemi pozuna dayanıyor olabilir.",
                throttle_duration_sec=2.0)

        rgb_w = cv_rgb.shape[1]
        if self.image_width != rgb_w:
            self.image_width = rgb_w
            self.current_f_l = self.base_f_l * (rgb_w / self.base_width)

        if self.latest_depth_frame is not None:
            scale_x = self.latest_depth_frame.shape[1] / rgb_w
            scale_y = self.latest_depth_frame.shape[0] / cv_rgb.shape[0]

        infer_start = time.perf_counter()
        results = self.model.predict(cv_rgb, conf=0.5, verbose=False)
        self._last_yolo_infer_ms = (time.perf_counter() - infer_start) * 1000.0

        for r in results[0].boxes:
            x1, y1, x2, y2 = map(int, r.xyxy[0])
            label = self.model.names[int(r.cls[0])]

            if self.latest_depth_frame is not None:
                uc, vc = (x1 + x2) // 2, (y1 + y2) // 2
                ud, vd = int(uc * scale_x), int(vc * scale_y)

                vd = max(10, min(self.latest_depth_frame.shape[0]-10, vd))
                ud = max(10, min(self.latest_depth_frame.shape[1]-10, ud))

                roi = self.latest_depth_frame[vd-10:vd+10, ud-10:ud+10]
                valid = roi[(roi > 100) & (roi < 10000)]
                if len(valid) > 0:
                    z = float(np.min(valid)) / 1000.0
                    ros_angle = -math.atan((uc - (rgb_w/2.0)) / self.current_f_l)
                    menzil = z / max(math.cos(ros_angle), 1e-3)

                    total_angle = self.fused_yaw + ros_angle
                    cam_x = self.ship_x + menzil * math.cos(total_angle)
                    cam_y = self.ship_y + menzil * math.sin(total_angle)
                    self._publish_buoy_detection(cam_x, cam_y, label)

            color = (0,255,0) if 'green' in label else (0,0,255) if 'red' in label else (0,255,255)
            cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), color, 2)

        try: self.yolo_img_pub.publish(self.bridge.cv2_to_imgmsg(cv_rgb, 'bgr8'))
        except Exception: pass

    def _publish_buoy_detection(self, cam_x: float, cam_y: float, label: str):
        """Ham renk adayını `/vision/buoy_detections` üzerine JSON olarak yayınlar —
        ID/pozisyon yaratmaz, sadece lidar_fusion_node'un renk-oyu füzyonuna girdi olur."""
        payload = json.dumps({'x': cam_x, 'y': cam_y, 'label': label})
        self.buoy_detections_pub.publish(String(data=payload))


def main():
    # bkz. lidar_fusion_node.main() — aynı SIGINT/shutdown güvenliği burada da uygulanıyor
    # (bu node'da can alıcı bir "durdur" komutu yok ama tutarlılık ve temiz kapanış için).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NjordCameraVisionNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
