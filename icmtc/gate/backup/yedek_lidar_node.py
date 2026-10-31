#!/usr/bin/env python3
"""
Njord YEDEK Sistemi — Kamerasız LiDAR+Pusula Gate Node'u
- primary/ (camera_vision_node.py + lidar_fusion_node.py, YOLO+LiDAR füzyonu) çökerse/kamera
  arızalanırsa diye BAĞIMSIZ bir yedek: SADECE LiDAR (/velodyne_points) + pusula
  (Pixhawk /mavros/local_position/odom VEYA OAK-D'nin /oak/imu/data'sı — GÖRÜNTÜ DEĞİL,
  sadece IMU/yönelim topic'i) kullanır. Hiçbir görüntü/renk işleme yapılmaz — ultralytics/
  cv_bridge importu YOKTUR, /oak/rgb/*, /oak/stereo/* topic'lerine hiç abone olunmaz.
- Duba renginin (kırmızı/yeşil) hangi tarafta olduğu KAMERA OLMADIĞI İÇİN doğrudan
  bilinemez — bunun yerine SAF GEOMETRİYE dayalı bir "gate çifti" testi kullanılır (bkz.
  yedek_common.find_gate_candidates): LiDAR'da gate genişliğine (1.5-3 m, gorev.jpg)
  uyan, gemiye yaklaşık eşit uzaklıkta, önde bir çift küme bulunca bunun gate olduğu
  varsayılır; sağ/sol (yeşil/kırmızı) etiketleme SADECE loglama içindir, direksiyon
  matematiği bu etikete bağlı değildir — sadece ORTA noktaya gidilir.
- Konum (ship_x, ship_y): AS'daki gibi Velodyne LiDAR'ın ardışık karelerindeki küme
  eşleştirmesinden ("LiDAR ego-motion") — GPS hiçbir şekilde kullanılmaz.
- AS'daki kalıcı çoklu-duba ID/renk-kilit sistemi (buoy_tracks, color_votes, locked,
  CONFIRMED_TRACK_RETENTION_S) BİLİNÇLİ OLARAK YOK — tek gate'lik pass/fail görev için
  gereksiz karmaşıklık. Onun yerine sadece küçük bir "aday gate + onay sayacı" durumu
  tutulur (bkz. _update_gate_candidate).
- Manuel evreli başlatma: '1' Mapping'i, '2' Algoritmayı başlatır/durdurur (toggle),
  'h' yerel (x,y metre) hedef girer, 'r' gate-aday hafızasını sıfırlar (AS ile AYNI
  tuşlar — operatör iki sistem arasında geçişte kafası karışmasın diye).
"""

import sys
import threading
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy,
                        QoSReliabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import OverrideRCIn
from tf2_ros import TransformBroadcaster
try:
    import termios
    import tty
except ImportError:  # Windows gibi tty'siz ortamlarda klavye dinleyici pasif kalır
    termios = None
    tty = None
import numpy as np

from yedek_common import (
    YEDEK_DETECTION_RADIUS_M, EGO_MATCH_RADIUS_M, MIN_EGO_MATCHES, MAX_SHIP_SPEED_MPS,
    MAX_SAFE_YAW_RATE_DEG_S,
    GATE_MIN_WIDTH_M, GATE_MAX_WIDTH_M, GATE_WIDTH_MARGIN_M, GATE_PAIR_DEPTH_TOLERANCE_M,
    GATE_CONFIRM_COUNT, GATE_MATCH_RADIUS_M, WAYPOINT_OFFSET_M, GOAL_TOLERANCE_M,
    MAX_LIDAR_POINTS_PER_SCAN,
    IMU_STALE_TIMEOUT_S, ODOM_STALE_TIMEOUT_S,
    STEER_KP, MAX_STEER_AXIS, MIN_THROTTLE_AXIS, KANAL_DUMEN, KANAL_GAZ, DUMEN_YONU,
    project_lidar_to_2d, cluster_lidar_points, cap_point_count, rotate_xy,
    estimate_translation, find_gate_candidates, classify_gate_sides,
    compute_yedek_gate_waypoint, calculate_pwm, mix_gaz_dumen,
)


class NjordYedekGateNode(Node):
    def __init__(self):
        super().__init__('njord_yedek_gate_node')

        self.get_logger().info("🟡 ROS 2 YEDEK (kamerasız LiDAR+pusula) Node'u Başlatılıyor...")

        # --- TEŞHİS BAYRAKLARI ---
        self.status = {'odom': False, 'imu': False, 'lidar': False}

        # --- ROBOT DURUMU (primary/lidar_fusion_node.py ile aynı LiDAR ego-motion mantığı) ---
        self.ship_x, self.ship_y, self.ship_yaw = 0.0, 0.0, 0.0
        self.fused_yaw = 0.0
        self.last_imu_time = 0.0
        self.alpha = 0.98
        self._orientation_mode = None

        # --- MANUEL EVRE KONTROLÜ ---
        self.mapping_active = False
        self.algorithm_active = False

        # --- GÖREV/MOTOR ---
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self._publish_neutral_rc()
        self.mission_goal = None
        self.mission_yaw = None
        self._reached_goal_logged = False

        self.tf_bc = TransformBroadcaster(self)
        self.ODOM_FRAME = 'odom'

        # --- LiDAR EGO-MOTION DURUMU ---
        self._prev_lidar_clusters_body = None
        self._prev_lidar_yaw = None
        self._last_yaw_rate_deg_s = 0.0

        # --- KAMERASIZ GATE-ADAY DURUMU (AS'daki kalıcı çoklu-duba ID sisteminin YERİNE,
        # kasıtlı olarak SADE bir "tek aday + onay sayacı" durumu — bkz. dosya başlığı) ---
        self._pending_gate_mid = None      # (x,y) — henüz GATE_CONFIRM_COUNT'a ulaşmamış aday
        self._pending_gate_count = 0
        self._confirmed_gate = None        # (mid_x, mid_y, dist, buoy_a, buoy_b) ya da None
        self._gate_lock = threading.RLock()

        # --- İSTATİSTİK ---
        self._distance_traveled_m = 0.0
        self._ego_speed_mps = 0.0
        self._ego_match_quality = 0.0
        self._last_lidar_wall_time = None
        self._last_odom_wall_time = None
        self._last_imu_wall_time = None

        # --- YAYINCILAR (minimal RViz — AS'daki tam dashboard/costmap YOK) ---
        self.marker_pub = self.create_publisher(MarkerArray, '/vision/markers', 10)
        self.lidar_filtered_pub = self.create_publisher(PointCloud2, '/lidar/filtered_points', 10)

        # --- CALLBACK GRUPLARI ---
        self.cb_fast_sensors = MutuallyExclusiveCallbackGroup()
        self.cb_lidar = MutuallyExclusiveCallbackGroup()
        self.cb_apf = MutuallyExclusiveCallbackGroup()
        self.cb_timers = MutuallyExclusiveCallbackGroup()

        # --- ABONELİKLER — SADECE LiDAR + pusula (odom/IMU). RGB/derinlik YOK. ---
        sensor_qos = qos_profile_sensor_data
        imu_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback,
                                  sensor_qos, callback_group=self.cb_fast_sensors)
        self.create_subscription(Imu, '/oak/imu/data', self.oak_imu_callback,
                                  imu_qos, callback_group=self.cb_fast_sensors)
        self.create_subscription(PointCloud2, '/velodyne_points', self.velodyne_callback,
                                  sensor_qos, callback_group=self.cb_lidar)

        # --- ZAMANLAYICILAR ---
        self.create_timer(0.2, self.publish_rviz_markers, callback_group=self.cb_timers)    # 5 Hz
        self.create_timer(2.0, self.print_diagnostic_status, callback_group=self.cb_timers) # 0.5 Hz
        self.create_timer(0.1, self._apf_timer_callback, callback_group=self.cb_apf)         # 10 Hz

    # -----------------------------------------------------------------------
    # TEŞHİS
    # -----------------------------------------------------------------------
    @staticmethod
    def _age_str(last_wall_time) -> str:
        if last_wall_time is None:
            return "veri yok"
        return f"{(time.time() - last_wall_time) * 1000.0:.0f} ms önce"

    def print_diagnostic_status(self):
        with self._gate_lock:
            gate = self._confirmed_gate
        gate_str = "YOK" if gate is None else f"({gate[0]:.1f},{gate[1]:.1f}) @ {gate[2]:.1f}m"
        status_msg = (
            f"\n--- YEDEK (kamerasız) NODE DURUMU ---\n"
            f"📍 Odom (Pixhawk, sadece yaw) : {'✅ AKTİF' if self.status['odom'] else '❌ BEKLENİYOR'} "
            f"(son veri: {self._age_str(self._last_odom_wall_time)})\n"
            f"🧭 IMU (OAK-D)    : {'✅ AKTİF' if self.status['imu'] else '❌ BEKLENİYOR'} "
            f"(son veri: {self._age_str(self._last_imu_wall_time)})\n"
            f"🛜 LiDAR          : {'✅ AKTİF' if self.status['lidar'] else '❌ BEKLENİYOR'}\n"
            f"🧭 Yönelim kaynağı: {self._orientation_mode or 'henüz veri yok'}\n"
            f"Konum (yerel, GPS'siz): ({self.ship_x:.1f}, {self.ship_y:.1f})  "
            f"Yaw: {math.degrees(self.fused_yaw):.1f}°\n"
            f"Ego-motion: hız {self._ego_speed_mps:.2f} m/s | eşleşme kalitesi "
            f"{self._ego_match_quality*100:.0f}% | kat edilen mesafe {self._distance_traveled_m:.1f} m\n"
            f"Kilitli gate adayı: {gate_str}\n"
            f"Mapping: {'AKTIF' if self.mapping_active else 'KAPALI'} | "
            f"Algoritma: {'AKTIF' if self.algorithm_active else 'KAPALI'}\n"
            f"-------------------"
        )
        self.get_logger().info(status_msg)
        for key in self.status:
            self.status[key] = False

    # -----------------------------------------------------------------------
    # SENSÖR CALLBACKLERİ — YÖNELİM (OAK IMU + Pixhawk, AS ile birebir aynı mantık,
    # tamamen kameradan bağımsız olduğu için değişmeden taşındı)
    # -----------------------------------------------------------------------
    def _imu_fresh(self, now: float) -> bool:
        return self._last_imu_wall_time is not None and (now - self._last_imu_wall_time) <= IMU_STALE_TIMEOUT_S

    def _odom_fresh(self, now: float) -> bool:
        return self._last_odom_wall_time is not None and (now - self._last_odom_wall_time) <= ODOM_STALE_TIMEOUT_S

    def _set_orientation_mode(self, mode: str):
        if mode == self._orientation_mode:
            return
        messages = {
            'fusion': "🧭 Yönelim kaynağı: OAK IMU + Pixhawk FÜZYON (ikisi de aktif)",
            'pixhawk_only': "🧭 Yönelim kaynağı: SADECE Pixhawk (OAK IMU yok/bayat) — füzyon devre dışı",
            'imu_only': "🧭 Yönelim kaynağı: SADECE OAK IMU (Pixhawk yok/bayat) — mutlak referans yok",
        }
        text = messages.get(mode, f"Yönelim kaynağı değişti: {mode}")
        if mode == 'fusion':
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        self._orientation_mode = mode

    def oak_imu_callback(self, msg: Imu):
        self.status['imu'] = True
        now = time.time()
        self._last_imu_wall_time = now

        dt = None if self.last_imu_time == 0.0 else (now - self.last_imu_time)
        self.last_imu_time = now
        odom_fresh = self._odom_fresh(now)

        if dt is None or dt > IMU_STALE_TIMEOUT_S:
            if odom_fresh:
                self.fused_yaw = self.ship_yaw
                self._set_orientation_mode('fusion')
            else:
                self._set_orientation_mode('imu_only')
            return

        gyro_z = msg.angular_velocity.z
        if odom_fresh:
            self.fused_yaw = self.alpha * (self.fused_yaw + gyro_z * dt) + (1.0 - self.alpha) * self.ship_yaw
            self._set_orientation_mode('fusion')
        else:
            self.fused_yaw += gyro_z * dt
            self._set_orientation_mode('imu_only')
        self.fused_yaw = math.atan2(math.sin(self.fused_yaw), math.cos(self.fused_yaw))

    def odom_callback(self, msg: Odometry):
        """SADECE yönelim (yaw) okunur — konumun tamamı LiDAR ego-motion'dan gelir."""
        self.status['odom'] = True
        now = time.time()
        self._last_odom_wall_time = now
        q = msg.pose.pose.orientation
        self.ship_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if not self._imu_fresh(now):
            self.fused_yaw = self.ship_yaw
            self._set_orientation_mode('pixhawk_only')

    # -----------------------------------------------------------------------
    # LiDAR CALLBACK
    # -----------------------------------------------------------------------
    def velodyne_callback(self, msg: PointCloud2):
        self.status['lidar'] = True
        if not self.mapping_active:
            return
        try:
            gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            xyz_list = [[p[0], p[1], p[2]] for p in gen]
        except Exception as e:
            self.get_logger().error(f"Velodyne okuma hatası: {e}")
            return
        if not xyz_list:
            return
        xyz = np.array(xyz_list, dtype=np.float32)

        points_2d = project_lidar_to_2d(xyz)
        if len(points_2d) > MAX_LIDAR_POINTS_PER_SCAN:
            self.get_logger().warn(
                f"[LiDAR] Bu taramada {len(points_2d)} nokta var (limit={MAX_LIDAR_POINTS_PER_SCAN}) — "
                f"rastgele altörnekleniyor.", throttle_duration_sec=2.0)
            points_2d = cap_point_count(points_2d, MAX_LIDAR_POINTS_PER_SCAN)
        clusters = cluster_lidar_points(points_2d)

        self._update_ego_motion(clusters)
        self._update_gate_candidate(clusters)
        self._broadcast_odom_tf()
        self._publish_filtered_lidar_cloud(points_2d)

    def _publish_filtered_lidar_cloud(self, points_2d: np.ndarray):
        if len(points_2d) == 0:
            return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')
        pts = [[float(x), float(y), 0.0] for x, y in points_2d]
        try:
            self.lidar_filtered_pub.publish(pc2.create_cloud_xyz32(header, pts))
        except Exception as e:
            self.get_logger().error(f"[lidar_filtered] {e}")

    # -----------------------------------------------------------------------
    # LiDAR EGO-MOTION (AS ile birebir aynı mantık, kameradan bağımsız)
    # -----------------------------------------------------------------------
    def _update_ego_motion(self, clusters: list):
        centers_body = [c['center'] for c in clusters]
        current_yaw = self.fused_yaw
        now = time.time()
        dt = (now - self._last_lidar_wall_time) if self._last_lidar_wall_time else None
        self._last_lidar_wall_time = now

        if self._prev_lidar_clusters_body and centers_body:
            dyaw = current_yaw - self._prev_lidar_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            if dt and dt > 0:
                self._last_yaw_rate_deg_s = abs(math.degrees(dyaw)) / dt
            dx_body, dy_body, matched = estimate_translation(
                self._prev_lidar_clusters_body, centers_body, dyaw)
            self._ego_match_quality = matched / max(len(centers_body), 1)

            if matched >= MIN_EGO_MATCHES:
                dx_world, dy_world = rotate_xy(dx_body, dy_body, current_yaw)
                step_m = math.hypot(dx_world, dy_world)
                implied_speed = (step_m / dt) if dt and dt > 0 else 0.0

                if implied_speed > MAX_SHIP_SPEED_MPS:
                    self.get_logger().warn(
                        f"[EGO-MOTION] İmkansız hız kestirimi ({implied_speed:.1f} m/s) — "
                        f"muhtemel yanlış eşleşme, bu kare reddedildi.", throttle_duration_sec=2.0)
                else:
                    self.ship_x += dx_world
                    self.ship_y += dy_world
                    self._distance_traveled_m += step_m
                    self._ego_speed_mps = implied_speed
            else:
                self._ego_speed_mps = 0.0
                self.get_logger().warn(
                    "[EGO-MOTION] Yetersiz küme eşleşmesi, öteleme bu karede atlandı.",
                    throttle_duration_sec=2.0)
        else:
            self._ego_match_quality = 0.0

        self._prev_lidar_clusters_body = centers_body
        self._prev_lidar_yaw = current_yaw

    def _broadcast_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ODOM_FRAME
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.ship_x
        t.transform.translation.y = self.ship_y
        t.transform.rotation.z = math.sin(self.fused_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.fused_yaw / 2.0)
        self.tf_bc.sendTransform(t)

    # -----------------------------------------------------------------------
    # KAMERASIZ GATE-ÇİFTİ TESPİTİ + DEBOUNCE/ONAY
    # -----------------------------------------------------------------------
    def _update_gate_candidate(self, clusters: list):
        """Her LiDAR taramasında en_yakın geçerli gate çiftini bulur (bkz.
        yedek_common.find_gate_candidates) ve GATE_CONFIRM_COUNT ardışık taramada
        (±GATE_MATCH_RADIUS_M içinde) tutarlı görülürse 'kilitli hedef' yapar. Hızlı
        dönüşte (bkz. MAX_SAFE_YAW_RATE_DEG_S) fused_yaw güvenilmez olduğu için bu
        turda hiç aday aranmaz (var olan kilitli hedef korunur, üzerine YAZILMAZ) —
        AS'daki 'arkada kalan duba birden yanımıza savruluyor' korumasıyla aynı fikir."""
        cos_y, sin_y = math.cos(self.fused_yaw), math.sin(self.fused_yaw)
        clusters_world = [
            (self.ship_x + lx * cos_y - ly * sin_y, self.ship_y + lx * sin_y + ly * cos_y)
            for (lx, ly) in (c['center'] for c in clusters)
        ]
        turning_fast = self._last_yaw_rate_deg_s > MAX_SAFE_YAW_RATE_DEG_S

        with self._gate_lock:
            if turning_fast:
                return  # var olan kilitli/bekleyen adaya dokunma, yeni arama yapma

            candidates = find_gate_candidates(clusters_world, (self.ship_x, self.ship_y), self.fused_yaw)
            best = candidates[0] if candidates else None

            if best is None:
                self._pending_gate_mid = None
                self._pending_gate_count = 0
                return

            mid = (best[0], best[1])
            if (self._pending_gate_mid is not None
                    and math.hypot(mid[0] - self._pending_gate_mid[0],
                                    mid[1] - self._pending_gate_mid[1]) <= GATE_MATCH_RADIUS_M):
                self._pending_gate_count += 1
            else:
                self._pending_gate_mid = mid
                self._pending_gate_count = 1

            if self._pending_gate_count >= GATE_CONFIRM_COUNT:
                if self._confirmed_gate is None:
                    left, right = classify_gate_sides(best[3], best[4], (self.ship_x, self.ship_y), self.fused_yaw)
                    self.get_logger().info(
                        f"[🔒 GATE] Kilitlendi @ ({best[0]:.1f},{best[1]:.1f}) {best[2]:.1f}m ileride — "
                        f"SOL(kırmızı? varsayım)=({left[0]:.1f},{left[1]:.1f}) "
                        f"SAĞ(yeşil? varsayım)=({right[0]:.1f},{right[1]:.1f})")
                self._confirmed_gate = best

    def reset_gate_memory(self):
        with self._gate_lock:
            self._pending_gate_mid = None
            self._pending_gate_count = 0
            self._confirmed_gate = None
        self.get_logger().info("🧹 Gate-aday hafızası sıfırlandı.")

    # -----------------------------------------------------------------------
    # RVIZ2 MARKER (minimal — AS'daki tam dashboard/costmap YOK)
    # -----------------------------------------------------------------------
    def publish_rviz_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = self.ODOM_FRAME

        clr = Marker()
        clr.header.frame_id = frame
        clr.header.stamp = stamp
        clr.action = Marker.DELETEALL
        marker_array.markers.append(clr)

        robot_m = Marker()
        robot_m.header.frame_id = frame
        robot_m.header.stamp = stamp
        robot_m.ns = 'robot'
        robot_m.id = 0
        robot_m.type = Marker.TRIANGLE_LIST
        robot_m.action = Marker.ADD
        robot_m.pose.position.x = float(self.ship_x)
        robot_m.pose.position.y = float(self.ship_y)
        robot_m.pose.position.z = 0.20
        robot_m.pose.orientation.z = math.sin(self.fused_yaw / 2.0)
        robot_m.pose.orientation.w = math.cos(self.fused_yaw / 2.0)
        robot_m.scale.x, robot_m.scale.y, robot_m.scale.z = 1.0, 1.0, 1.0
        robot_m.color.r, robot_m.color.g, robot_m.color.b, robot_m.color.a = 1.0, 0.65, 0.0, 0.9  # turuncu: YEDEK ayırt edilsin
        robot_m.points = [
            Point(x=0.6, y=0.0, z=0.0),
            Point(x=-0.4, y=0.35, z=0.0),
            Point(x=-0.4, y=-0.35, z=0.0),
        ]
        marker_array.markers.append(robot_m)

        arrow_m = Marker()
        arrow_m.header.frame_id = frame
        arrow_m.header.stamp = stamp
        arrow_m.ns = 'heading'
        arrow_m.id = 1
        arrow_m.type = Marker.ARROW
        arrow_m.action = Marker.ADD
        arrow_m.scale.x, arrow_m.scale.y, arrow_m.scale.z = 0.15, 0.30, 0.30
        arrow_m.color.r, arrow_m.color.g, arrow_m.color.b, arrow_m.color.a = 1.0, 0.1, 0.1, 1.0
        p0 = Point(x=float(self.ship_x), y=float(self.ship_y), z=0.35)
        p1 = Point(
            x=float(self.ship_x + 3.5 * math.cos(self.fused_yaw)),
            y=float(self.ship_y + 3.5 * math.sin(self.fused_yaw)), z=0.35)
        arrow_m.points = [p0, p1]
        marker_array.markers.append(arrow_m)

        with self._gate_lock:
            gate = self._confirmed_gate

        if gate is not None:
            mid_x, mid_y, _, buoy_a, buoy_b = gate
            for idx, (bx, by) in enumerate((buoy_a, buoy_b)):
                b_m = Marker()
                b_m.header.frame_id = frame
                b_m.header.stamp = stamp
                b_m.ns = 'gate_buoys'
                b_m.id = idx
                b_m.type = Marker.SPHERE
                b_m.action = Marker.ADD
                b_m.pose.position.x, b_m.pose.position.y, b_m.pose.position.z = float(bx), float(by), 0.3
                b_m.pose.orientation.w = 1.0
                b_m.scale.x = b_m.scale.y = b_m.scale.z = 0.4
                b_m.color.r, b_m.color.g, b_m.color.b, b_m.color.a = 0.6, 0.6, 0.6, 0.95  # renk BİLİNMİYOR -> gri
                marker_array.markers.append(b_m)

            mid_m = Marker()
            mid_m.header.frame_id = frame
            mid_m.header.stamp = stamp
            mid_m.ns = 'gate_midpoint'
            mid_m.id = 0
            mid_m.type = Marker.SPHERE
            mid_m.action = Marker.ADD
            mid_m.pose.position.x, mid_m.pose.position.y, mid_m.pose.position.z = float(mid_x), float(mid_y), 0.3
            mid_m.pose.orientation.w = 1.0
            mid_m.scale.x = mid_m.scale.y = mid_m.scale.z = 0.25
            mid_m.color.r, mid_m.color.g, mid_m.color.b, mid_m.color.a = 1.0, 0.65, 0.0, 0.95
            marker_array.markers.append(mid_m)

        dashboard = Marker()
        dashboard.header.frame_id = frame
        dashboard.header.stamp = stamp
        dashboard.ns = 'dashboard'
        dashboard.id = 3
        dashboard.type = Marker.TEXT_VIEW_FACING
        dashboard.action = Marker.ADD
        dashboard.pose.position.x = self.ship_x + 6.0
        dashboard.pose.position.y = self.ship_y + 6.0
        dashboard.pose.position.z = 2.2
        dashboard.pose.orientation.w = 1.0
        dashboard.scale.z = 0.35
        dashboard.color.r = dashboard.color.g = dashboard.color.b = dashboard.color.a = 1.0
        gate_str = "YOK" if gate is None else f"{gate[2]:.1f}m ileride"
        dashboard.text = (
            f"[YEDEK / KAMERASIZ] Konum: ({self.ship_x:.1f}, {self.ship_y:.1f})  "
            f"Yaw: {math.degrees(self.fused_yaw):.0f}°\n"
            f"Hiz: {self._ego_speed_mps:.2f} m/s | Eslesme: {self._ego_match_quality*100:.0f}%\n"
            f"Gate: {gate_str}\n"
            f"Mapping: {'AKTIF' if self.mapping_active else 'KAPALI'} | "
            f"Algoritma: {'AKTIF' if self.algorithm_active else 'KAPALI'}"
        )
        marker_array.markers.append(dashboard)

        self.marker_pub.publish(marker_array)

    # -----------------------------------------------------------------------
    # GATE SÜRÜŞÜ + RC-OVERRIDE MOTOR ÇIKIŞI (AS ile aynı P-kontrol iskeleti)
    # -----------------------------------------------------------------------
    def _publish_neutral_rc(self):
        msg = OverrideRCIn()
        msg.channels = [65535] * 18
        msg.channels[KANAL_DUMEN] = calculate_pwm(0.0)
        msg.channels[KANAL_GAZ] = calculate_pwm(0.0)
        self.rc_pub.publish(msg)

    def _apf_timer_callback(self):
        if not self.algorithm_active:
            return
        self._gate_drive_hesapla()

    def _gate_drive_hesapla(self):
        """primary/lidar_fusion_node._gate_drive_hesapla ile AYNI SÖZLEŞME: onaylı bir gate
        adayı varsa orta noktasına (+ WAYPOINT_OFFSET_M) sürülür, yoksa doğrudan
        mission_goal'e (pusula/operatör hedefi) doğru düz gidilir. Hedef girilmemişse
        veya hedefe ulaşılmışsa motorlar nötre çekilir — gate geçildikten sonra ayrı bir
        'dur' mantığı YOK, sistem mission_goal'e ulaşana kadar sürmeye devam eder."""
        if self.mission_goal is None:
            self._publish_neutral_rc()
            return

        gx, gy = self.mission_goal
        if math.hypot(gx - self.ship_x, gy - self.ship_y) < GOAL_TOLERANCE_M:
            self._publish_neutral_rc()
            if not self._reached_goal_logged:
                self.get_logger().info(f"🏁 HEDEFE ULAŞILDI! ({gx:.1f}, {gy:.1f})")
                self._reached_goal_logged = True
            return
        self._reached_goal_logged = False

        with self._gate_lock:
            gate = self._confirmed_gate

        waypoint = compute_yedek_gate_waypoint((self.ship_x, self.ship_y), self.fused_yaw, gate)
        target = waypoint if waypoint is not None else self.mission_goal

        bearing_to_target = math.atan2(target[1] - self.ship_y, target[0] - self.ship_x)
        heading_error = bearing_to_target - self.fused_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        turn = max(-MAX_STEER_AXIS, min(MAX_STEER_AXIS, STEER_KP * heading_error))
        throttle = max(MIN_THROTTLE_AXIS, 1.0 - abs(turn))
        sol = max(-1.0, min(1.0, throttle - turn))
        sag = max(-1.0, min(1.0, throttle + turn))

        gaz, dumen = mix_gaz_dumen(sol, sag)
        msg = OverrideRCIn()
        msg.channels = [65535] * 18
        msg.channels[KANAL_DUMEN] = calculate_pwm(dumen)
        msg.channels[KANAL_GAZ] = calculate_pwm(gaz)
        self.rc_pub.publish(msg)


# -----------------------------------------------------------------------
# MANUEL EVRE KONTROLÜ — KLAVYE DİNLEYİCİ (AS ile aynı tuşlar)
# -----------------------------------------------------------------------
def _keypress_listener(node: 'NjordYedekGateNode'):
    """'1' Mapping'i, '2' Algoritmayı başlatır/durdurur (toggle), 'h' yerel (x,y) hedef
    girer, 'r' gate-aday hafızasını sıfırlar (primary/lidar_fusion_node._keypress_listener
    ile birebir aynı UX — operatör iki sistem arasında geçişte kafası karışmasın)."""
    if termios is None or tty is None or not sys.stdin.isatty():
        node.get_logger().warn(
            "⌨️  Klavye dinleyici başlatılamadı (tty yok) — mapping/algoritma "
            "varsayılan olarak KAPALI kalacak.")
        return

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        node.get_logger().info(
            "⌨️  [1] Mapping başlat/durdur   [2] Algoritma başlat/durdur   "
            "[h] Hedef gir (yerel x,y metre)   [r] Gate hafızasını sıfırla   (Ctrl+C: çıkış)")
        while rclpy.ok():
            ch = sys.stdin.read(1)
            if ch == '1':
                node.mapping_active = not node.mapping_active
                node.get_logger().info(f"🗺️  Mapping {'BAŞLADI' if node.mapping_active else 'DURDU'}")
                if not node.mapping_active:
                    node.algorithm_active = False
                    node._publish_neutral_rc()
            elif ch == '2':
                if not node.mapping_active:
                    node.get_logger().warn("⚠️  Algoritma, Mapping kapalıyken başlatılamaz — önce '1'e bas.")
                    continue
                node.algorithm_active = not node.algorithm_active
                node.get_logger().info(f"🚀 Algoritma {'BAŞLADI' if node.algorithm_active else 'DURDU'}")
                if not node.algorithm_active:
                    node._publish_neutral_rc()
            elif ch == 'h':
                node.algorithm_active = False
                node._publish_neutral_rc()
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                try:
                    print("\n=== HEDEF GİRİŞİ (gemi başlangıç noktasına göre YEREL metre) ===")
                    raw = input("Hedef (B) [x y]: ").strip().split()
                    gx, gy = float(raw[0]), float(raw[1])
                    node.mission_goal = (gx, gy)
                    node.mission_yaw = math.atan2(gy - node.ship_y, gx - node.ship_x)
                    node._reached_goal_logged = False
                    node.get_logger().info(
                        f"🎯 Hedef ayarlandı: ({gx:.1f}, {gy:.1f}) — görev yönü: "
                        f"{math.degrees(node.mission_yaw):.1f}°")
                except Exception:
                    print("[HATA] Geçersiz giriş! Format: 'x y' (örn: 50.0 10.0)")
                finally:
                    tty.setcbreak(fd)
            elif ch == 'r':
                if not node.mapping_active:
                    node.get_logger().warn("⚠️  Mapping kapalıyken sıfırlanacak bir şey yok.")
                    continue
                node.reset_gate_memory()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NjordYedekGateNode()
    listener = threading.Thread(target=_keypress_listener, args=(node,), daemon=True)
    listener.start()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_neutral_rc()
        except Exception as e:
            node.get_logger().error(f"⚠️  GÜVENLİK DURDURMA KOMUTU GÖNDERİLEMEDİ: {e}")
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
