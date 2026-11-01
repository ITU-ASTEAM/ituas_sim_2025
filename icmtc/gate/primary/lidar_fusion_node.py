#!/usr/bin/env python3
"""
Njord Füzyon Sistemi v8.0 — LiDAR Ego-Motion + Kalıcı Duba ID (LiDAR/APF/RViz Node'u)
- Bu dosya eski tek-parça `oak.py`'nin LiDAR + konumlandırma + duba takibi + APF sürüş +
  RViz/costmap yayınlarını içerir. YOLO+derinlik füzyonu artık AYRI bir process'te
  çalışan `camera_vision_node.py`'dedir — ikisi birlikte `calistir.py` ile başlatılır.
  Bu bölünmenin nedeni: YOLO inference'ı (Python/GIL ağırlıklı) bu node'un LiDAR
  callback'ini asla bloklamasın, ayrıca Jetson Orin NX'in çekirdekleri iki gerçek OS
  process'i arasında paylaşılsın (threading yerine gerçek paralellik).
- OAK-D IMU + Pixhawk pusulası füzyonu (Complementary Filter) — SADECE yönelim (yaw)
- Konum (ship_x, ship_y): Pixhawk odom'dan DEĞİL, Velodyne LiDAR'ın ardışık karelerindeki
  küme eşleştirmesinden ("LiDAR ego-motion") — GPS hiçbir şekilde kullanılmaz.
- Duba kimliği (ID): LiDAR kümesi rolling-tracker'ı ID otoritesidir; kamera+derinlik
  tespiti `/vision/buoy_detections` topic'i üzerinden gelir ve sadece en yakın LiDAR
  track'ine renk oyu ekler (ID/pozisyon YARATMAZ) — en yakın iki aday belirsizse
  (birbirine çok yakınsa) hiçbirine oy verilmez (bkz. find_unambiguous_nearest).
- Kalıcı Duba ID: SADECE kamera tarafından RENK KİLİTLENMİŞ track'ler kalıcıdır — yaşı
  veya gemiye göre konumu ne olursa olsun CONFIRMED_TRACK_RETENTION_S (çok uzun ama
  SONLU, örn. 5dk) süresince silinmez. Rengi kilitlenmemiş track'ler (sırf LiDAR'da
  tekrar görülmüş olsa bile) STALE_TRACK_MAX_AGE_S ile kısa sürede temizlenir — aksi
  halde gürültü (dalga/köpük) haritayı sonsuza dek şişirir. Algılama alanı (20m yarıçap)
  sadece hangi YENİ kümelerin işleme dahil edileceğini belirler.
- Manuel evreli başlatma: '1' Mapping'i, '2' Algoritmayı başlatır/durdurur (toggle),
  'h' yerel (x,y metre) hedef girer, 'r' haritayı sıfırlar.
- Sürüş: Nav2/GPS YOK. Gerçek Jetson testinde teknenin `/cmd_vel` (Twist) DEĞİL,
  `/mavros/rc/override` (OverrideRCIn) ile sürüldüğü anlaşıldı — bu yüzden sürüş çıkışı
  RC-override script'indeki mixing/PWM matematiğini kullanır (bkz. oak_common.
  calculate_pwm/mix_gaz_dumen). Karar mantığı waypoint_generator.py'nin kapı-geçiş
  algoritmasından uyarlandı: SARI=orta/engel, KIRMIZI+YEŞİL (havuzlanmış)=kenar/gate —
  önde bir sarı varsa geniş kenar tarafına ara hedef konur (bkz. compute_gate_waypoint),
  her tick'te yeniden hesaplanıp sürekli bir P-kontrolle sürülür (Nav2 action yok).
"""

from collections import defaultdict
import json
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
from std_msgs.msg import Header, Bool, String
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
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

from oak_common import (
    MAP_RADIUS_M, EGO_MATCH_RADIUS_M, MIN_EGO_MATCHES, MAX_SHIP_SPEED_MPS,
    MAX_SAFE_YAW_RATE_DEG_S,
    TRACK_MATCH_RADIUS_M, STALE_TRACK_MAX_AGE_S, MAX_TRACK_CONFIDENCE,
    CONFIRMED_TRACK_MIN_CONFIDENCE, CONFIRMED_TRACK_RETENTION_S, MAX_TOTAL_TRACKS,
    FUSION_DISTANCE_M, FUSION_AMBIGUITY_MARGIN_M, COLOR_VOTE_THRESHOLD,
    MAX_LIDAR_POINTS_PER_SCAN, COSTMAP_RES, COSTMAP_SIZE,
    IMU_STALE_TIMEOUT_S, ODOM_STALE_TIMEOUT_S,
    LOOK_AHEAD_M, WAYPOINT_OFFSET_M, GOAL_TOLERANCE_M, MIN_SAFE_DIST_M, SIDE_EDGE_MAX_M,
    STEER_KP, MAX_STEER_AXIS, MIN_THROTTLE_AXIS, KANAL_DUMEN, KANAL_GAZ, DUMEN_YONU,
    point_in_range, project_lidar_to_2d, cluster_lidar_points, cap_point_count,
    rotate_xy, estimate_translation, find_unambiguous_nearest, apply_track_position,
    compute_gate_waypoint, calculate_pwm, mix_gaz_dumen,
)


class NjordLidarFusionNode(Node):
    def __init__(self):
        super().__init__('njord_lidar_fusion_node')

        self.get_logger().info("🟢 ROS 2 LiDAR Füzyon Node'u Başlatılıyor...")

        # --- TEŞHİS BAYRAKLARI (rgb/depth/yolo artık camera_vision_node'da) ---
        self.status = {'odom': False, 'imu': False, 'lidar': False}

        # --- ROBOT DURUMU ---
        # ship_x/ship_y ARTIK GPS/mavros pozisyonundan gelmiyor — sadece LiDAR ego-motion
        # ile node başlangıcına (0,0) göre biriktirilen tamamen yerel bir konum (bkz.
        # velodyne_callback -> _update_ego_motion). ship_yaw ise Pixhawk odom'unun SADECE
        # orientation alanından okunur (pusula/IMU kaynaklı, GPS değil).
        self.ship_x, self.ship_y, self.ship_yaw = 0.0, 0.0, 0.0
        self.fused_yaw = 0.0
        self.last_imu_time = 0.0
        self.alpha = 0.98  # IMU ve Pusula füzyon katsayısı
        # Yönelim kaynağı hangi sensör(ler) mevcutsa ona göre otomatik seçilir: ikisi de
        # taze ise füzyon, sadece biri taze ise o TEK BAŞINA kullanılır (asla donuk
        # kalınmaz), hiçbiri taze değilse son bilinen değerde kalır. Mod değişimi
        # loglanır (bkz. _set_orientation_mode) — spam olmaması için sadece geçişte.
        self._orientation_mode = None

        # --- MANUEL EVRE KONTROLÜ ---
        self.mapping_active = False
        self.algorithm_active = False

        # --- GATE-GEÇİŞ NAVİGASYONU + MOTOR ÇIKIŞI ---
        # Gerçek Jetson testinde /cmd_vel'in hiçbir şeyi sürmediği, tekneyi asıl
        # /mavros/rc/override'ın (OverrideRCIn) sürdüğü anlaşıldı — RC-override
        # script'indeki mixing/PWM matematiği kullanılıyor (bkz. oak_common.calculate_pwm
        # / mix_gaz_dumen). Nav2/GPS YOK: hedef klavyeden yerel (x,y) olarak girilir,
        # her kontrol tokunda (10Hz) waypoint yeniden hesaplanıp sürekli sürülür.
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self._publish_neutral_rc()  # başlangıçta motorlar kesin durur
        self.mission_goal = None   # (x,y) yerel, gemi başlangıç noktasına göre
        self.mission_yaw = None    # başlangıçtan hedefe bearing — _in_front_of referansı
        self._goal_input_pending = False   # 'h' basıldığında klavye thread'i input() bekliyor
        self._reached_goal_logged = False  # hedefe ulaşma logunun tekrarını önler

        self.tf_bc = TransformBroadcaster(self)
        self.ODOM_FRAME = 'odom'  # global/GPS iddiası olmadığı için 'map' değil 'odom'

        # --- LiDAR EGO-MOTION DURUMU ---
        self._prev_lidar_clusters_body = None   # bir önceki taramanın küme merkezleri (gövde çerçevesi)
        self._prev_lidar_yaw = None              # o taramadaki fused_yaw (dyaw hesaplamak için)
        # Anlık dönüş hızı (derece/sn) — hızlı dönüşte var olan duba track'lerinin
        # konumunu güncellemeyi durdurmak için kullanılır (bkz. _update_buoy_tracks).
        self._last_yaw_rate_deg_s = 0.0

        # --- KALICI DUBA TAKİBİ (ID otoritesi LiDAR'da) ---
        self.buoy_tracks: list = []   # {id,x,y,last_seen,confidence,color_votes,color,locked}
        self._next_track_id = 1
        # buoy_tracks LiDAR (cb_lidar) ve kamera-tespiti-aboneliği (cb_vision_sub)
        # thread'lerinden AYNI ANDA mutasyona uğrayabiliyor — bu kilit iki thread'in aynı
        # fiziksel dubaya YANLIŞLIKLA iki farklı ID vermesini engeller.
        self._tracks_lock = threading.RLock()

        # --- İSTATİSTİK / GECİKME TAKİBİ ---
        self._distance_traveled_m = 0.0
        self._ego_speed_mps = 0.0
        self._ego_match_quality = 0.0     # son öteleme kestiriminde eşleşen küme oranı [0,1]
        self._last_lidar_wall_time = None
        self._last_odom_wall_time = None
        self._last_imu_wall_time = None

        # --- YAYINCILAR ---
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 10)
        self.costmap_res, self.costmap_size = COSTMAP_RES, COSTMAP_SIZE

        self.marker_pub = self.create_publisher(MarkerArray, '/vision/markers', 10)
        self.lidar_filtered_pub = self.create_publisher(PointCloud2, '/lidar/filtered_points', 10)

        # camera_vision_node'un ihtiyaç duyduğu gemi pozu — 2-process mimaride paylaşılan
        # Python state artık yok, bu yüzden pozu topic üzerinden yayınlıyoruz.
        self.ego_pose_pub = self.create_publisher(Odometry, '/njord/ego_pose', 10)

        # mapping_active durumu da aynı sebeple topic'e taşındı. TRANSIENT_LOCAL: geç
        # başlayan camera_vision_node, bir sonraki '1' tuşunu beklemeden son durumu alır.
        mapping_state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.mapping_active_pub = self.create_publisher(Bool, '/njord/mapping_active', mapping_state_qos)
        self.mapping_active_pub.publish(Bool(data=False))

        # --- CALLBACK GRUPLARI (modüler/asenkron yürütme) ---
        # Her alt sistem KENDİ grubunda: bir grup içindeki callback'ler birbirini bekler
        # (Mutually Exclusive) ama FARKLI gruplar MultiThreadedExecutor sayesinde PARALEL
        # çalışır. APF kendi grubunda olduğu için RViz/costmap yayınları sürüş kontrolünü
        # geciktirmez.
        self.cb_fast_sensors = MutuallyExclusiveCallbackGroup()
        self.cb_lidar = MutuallyExclusiveCallbackGroup()
        self.cb_apf = MutuallyExclusiveCallbackGroup()
        self.cb_timers = MutuallyExclusiveCallbackGroup()
        self.cb_vision_sub = MutuallyExclusiveCallbackGroup()

        # --- ABONELİKLER ---
        sensor_qos = qos_profile_sensor_data
        # OAK-D IMU'nun (depthai-ros) yayıncısı RELIABLE — bizim varsayılan BEST_EFFORT
        # isteğimizle DDS spesifikasyonuna göre uyumlu olması gerekse de (RELIABLE
        # yayıncı + BEST_EFFORT abone teorik olarak eşleşir), gerçek Jetson testinde
        # mesaj HİÇ ulaşmadı (muhtemelen RMW'ye özgü bir tuhaflık). Yayıncıyla BİREBİR
        # aynı QoS'u isteyerek bunu kesin şekilde çözüyoruz.
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
        self.create_subscription(String, '/vision/buoy_detections', self.buoy_detection_callback,
                                  10, callback_group=self.cb_vision_sub)

        # --- ZAMANLAYICILAR ---
        self.create_timer(0.5, self.publish_local_costmap, callback_group=self.cb_timers)   # 2 Hz
        self.create_timer(0.2, self.publish_rviz_markers, callback_group=self.cb_timers)    # 5 Hz
        self.create_timer(2.0, self.print_diagnostic_status, callback_group=self.cb_timers) # 0.5 Hz
        self.create_timer(0.05, self.publish_ego_pose, callback_group=self.cb_timers)       # 20 Hz
        self.create_timer(0.1, self._apf_timer_callback, callback_group=self.cb_apf)         # 10 Hz

    # -----------------------------------------------------------------------
    # TEŞHİS VE LOGLAMA
    # -----------------------------------------------------------------------
    @staticmethod
    def _age_str(last_wall_time) -> str:
        if last_wall_time is None:
            return "veri yok"
        return f"{(time.time() - last_wall_time) * 1000.0:.0f} ms önce"

    def print_diagnostic_status(self):
        kilitli = sum(1 for t in self.buoy_tracks if t['locked'])
        status_msg = (
            f"\n--- LiDAR/APF NODE DURUMU ---\n"
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
            f"Haritadaki Duba Sayısı: {len(self.buoy_tracks)} (kilitli: {kilitli})\n"
            f"Mapping: {'AKTIF' if self.mapping_active else 'KAPALI'} | "
            f"Algoritma: {'AKTIF' if self.algorithm_active else 'KAPALI'}\n"
            f"-------------------"
        )
        self.get_logger().info(status_msg)
        for key in self.status:
            self.status[key] = False

    # -----------------------------------------------------------------------
    # SENSÖR CALLBACKLERİ (YÖNELİM: OAK IMU + Pixhawk, HANGİSİ VARSA O KULLANILIR)
    # -----------------------------------------------------------------------
    def _imu_fresh(self, now: float) -> bool:
        return self._last_imu_wall_time is not None and (now - self._last_imu_wall_time) <= IMU_STALE_TIMEOUT_S

    def _odom_fresh(self, now: float) -> bool:
        return self._last_odom_wall_time is not None and (now - self._last_odom_wall_time) <= ODOM_STALE_TIMEOUT_S

    def _set_orientation_mode(self, mode: str):
        """Yönelim kaynağı değiştiğinde (füzyon <-> tek-kaynak) BİR KEZ loglar — her
        mesajda değil, sadece geçişte (spam olmasın diye)."""
        if mode == self._orientation_mode:
            return
        messages = {
            'fusion': "🧭 Yönelim kaynağı: OAK IMU + Pixhawk FÜZYON (ikisi de aktif)",
            'pixhawk_only': "🧭 Yönelim kaynağı: SADECE Pixhawk (OAK IMU yok/bayat) — füzyon devre dışı, veri donuk kalmıyor",
            'imu_only': "🧭 Yönelim kaynağı: SADECE OAK IMU (Pixhawk yok/bayat) — mutlak referans yok, zamanla kayabilir",
        }
        text = messages.get(mode, f"Yönelim kaynağı değişti: {mode}")
        # rclpy aynı çağrı satırında (call-site) .info/.warn arasında dinamik geçişe
        # izin vermiyor ("Logger severity cannot be changed between calls.") — bu yüzden
        # iki seviyeyi AYRI satırlarda, sabit şekilde çağırıyoruz.
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
            # Filtre ilk kez çalışıyor VEYA uzun bir aradan sonra devam ediyor — eski
            # dt ile jiroskop entegrasyonu yapmak yaw'da ani bir sıçramaya yol açar,
            # bu yüzden entegrasyonu atlayıp sıfırdan başlıyoruz.
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
            # Pixhawk yok/bayat -> SADECE jiroskop entegrasyonu. Mutlak referans yok
            # (zamanla kayabilir) ama kullanıcı isteği gereği yönelim verisi VARSA
            # mutlaka kullanılır, asla donuk kalınmaz.
            self.fused_yaw += gyro_z * dt
            self._set_orientation_mode('imu_only')
        self.fused_yaw = math.atan2(math.sin(self.fused_yaw), math.cos(self.fused_yaw))

    def odom_callback(self, msg: Odometry):
        """SADECE yönelim (yaw) okunur. Pozisyon (pose.pose.position) BİLEREK okunmuyor —
        konumun tamamı LiDAR ego-motion'dan (velodyne_callback) geliyor."""
        self.status['odom'] = True
        now = time.time()
        self._last_odom_wall_time = now
        q = msg.pose.pose.orientation
        self.ship_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if not self._imu_fresh(now):
            # OAK IMU yok/bayat -> fused_yaw DOĞRUDAN Pixhawk'tan (füzyon değil,
            # pass-through) — IMU hiç gelmese bile yönelim asla donuk kalmaz.
            self.fused_yaw = self.ship_yaw
            self._set_orientation_mode('pixhawk_only')

    def velodyne_callback(self, msg: PointCloud2):
        self.status['lidar'] = True
        if not self.mapping_active:
            return  # '1'e basılmadan LiDAR işlenmez, ego-motion/track birikmez
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
                f"performans güvenliği için rastgele {MAX_LIDAR_POINTS_PER_SCAN} noktaya altörnekleniyor.",
                throttle_duration_sec=2.0)
            points_2d = cap_point_count(points_2d, MAX_LIDAR_POINTS_PER_SCAN)
        clusters = cluster_lidar_points(points_2d)

        self._update_ego_motion(clusters)
        self._update_buoy_tracks(clusters)
        self._broadcast_odom_tf()
        self._publish_filtered_lidar_cloud(points_2d)

    def _publish_filtered_lidar_cloud(self, points_2d: np.ndarray):
        """Ham (gürültülü, 360° çok yoğun) Velodyne bulutu yerine, z-bandı + dairesel
        algılama-alanı filtresinden GEÇMİŞ noktaları ayrı bir topic'te yayınlar."""
        if len(points_2d) == 0:
            return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')
        pts = [[float(x), float(y), 0.0] for x, y in points_2d]
        try:
            self.lidar_filtered_pub.publish(pc2.create_cloud_xyz32(header, pts))
        except Exception as e:
            self.get_logger().error(f"[lidar_filtered] {e}")

    # -----------------------------------------------------------------------
    # LiDAR EGO-MOTION (GPS'siz "optical flow" karşılığı — kare-arası küme eşleştirme)
    # -----------------------------------------------------------------------
    def _update_ego_motion(self, clusters: list):
        centers_body = [c['center'] for c in clusters]
        current_yaw = self.fused_yaw
        now = time.time()
        dt = (now - self._last_lidar_wall_time) if self._last_lidar_wall_time else None
        self._last_lidar_wall_time = now

        if self._prev_lidar_clusters_body and centers_body:
            dyaw = current_yaw - self._prev_lidar_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # -180/+180 sarmasını doğru al
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
                        f"[EGO-MOTION] İmkansız hız kestirimi ({implied_speed:.1f} m/s > "
                        f"{MAX_SHIP_SPEED_MPS:.1f} m/s) — muhtemel yanlış eşleşme, bu kare reddedildi.",
                        throttle_duration_sec=2.0)
                else:
                    self.ship_x += dx_world
                    self.ship_y += dy_world
                    self._distance_traveled_m += step_m
                    self._ego_speed_mps = implied_speed
                    self.get_logger().debug(
                        f"[EGO-MOTION] dx={dx_world:+.3f} dy={dy_world:+.3f} m | "
                        f"hiz={implied_speed:.2f} m/s | eslesme={matched}/{len(centers_body)}")
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

    def publish_ego_pose(self):
        """camera_vision_node'un derinlik+açı hesabı için ihtiyaç duyduğu gemi pozunu
        (ship_x, ship_y, fused_yaw) topic üzerinden yayınlar — process'ler arası paylaşılan
        Python state olmadığı için bu, iki process'i senkron tutan tek kanaldır."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.ODOM_FRAME
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(self.ship_x)
        msg.pose.pose.position.y = float(self.ship_y)
        msg.pose.pose.orientation.z = math.sin(self.fused_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.fused_yaw / 2.0)
        self.ego_pose_pub.publish(msg)

    # -----------------------------------------------------------------------
    # KALICI DUBA ID TAKİBİ (ID otoritesi LiDAR kümeleri) + KAMERA RENK FÜZYONU
    # -----------------------------------------------------------------------
    def _update_buoy_tracks(self, clusters: list):
        """Hızlı dönüş sırasında (bkz. MAX_SAFE_YAW_RATE_DEG_S) fused_yaw'ın o anki değeri
        güvenilmez sayılır — VAR OLAN track'lerin konumu bu tur sırasında GÜNCELLENMEZ
        (donuk kalır, silinmez), sadece last_seen/confidence tazelenir. Bu, hızlı/sert
        dönüşlerde 'arkada kalan duba birden yanımıza savruluyor' sorununun birincil
        çözümüdür — YENİ track oluşturma bundan etkilenmez (görülen her şey kaydedilir,
        sadece zaten bilinen bir dubanın üzerine YANLIŞ konum yazılmaz).
        Dönüş güvenli hızdaysa bile normal küçük ölçüm gürültüsü hemen kabul edilir, ama
        fiziksel olarak imkansız büyük bir sıçrama yine de anlık kabul edilmez (bkz.
        apply_track_position / MAX_TRACK_JUMP_M) — ikinci bir güvenlik ağı."""
        now = time.time()
        cos_y, sin_y = math.cos(self.fused_yaw), math.sin(self.fused_yaw)
        turning_fast = self._last_yaw_rate_deg_s > MAX_SAFE_YAW_RATE_DEG_S

        with self._tracks_lock:
            for c in clusters:
                lx, ly = c['center']
                wx = self.ship_x + lx * cos_y - ly * sin_y
                wy = self.ship_y + lx * sin_y + ly * cos_y

                track = self._match_or_create_track(wx, wy, now)
                # Yeni oluşturulan bir track'in confidence'ı henüz 0'dır (bkz.
                # _match_or_create_track) — bu yüzden confidence>0, "bu track daha önceki
                # bir taramadan beri var" anlamına gelir (bu taramada YENİ değil).
                is_preexisting = track['confidence'] > 0
                if turning_fast and is_preexisting:
                    pass  # hızlı dönüşte var olan track'in konumuna dokunma
                else:
                    apply_track_position(track, wx, wy)
                track['last_seen'] = now
                track['confidence'] = min(track['confidence'] + 1, MAX_TRACK_CONFIDENCE)

            self._cleanup_stale_tracks(now)

    def _match_or_create_track(self, wx: float, wy: float, now: float) -> dict:
        """Yeni LiDAR küme konumunu var olan track'lerle eşleştirir (mekânsal süreklilik);
        eşleşme yoksa YENİ ID verir."""
        best_track, best_d = None, float('inf')
        for t in self.buoy_tracks:
            d = math.hypot(t['x'] - wx, t['y'] - wy)
            if d < best_d:
                best_d, best_track = d, t

        if best_track is not None and best_d < TRACK_MATCH_RADIUS_M:
            return best_track

        new_track = {
            'id': self._next_track_id,
            'x': wx, 'y': wy,
            'last_seen': now,
            'confidence': 0,
            'color_votes': defaultdict(int),
            'color': None,
            'locked': False,
            # apply_track_position'ın sıçrama-onay mekanizması için (bkz. oak_common):
            'pending_x': None, 'pending_y': None, 'pending_count': 0,
        }
        self._next_track_id += 1
        self.buoy_tracks.append(new_track)
        self.get_logger().info(f"[+ID] Yeni duba track'i #{new_track['id']} @ ({wx:.1f},{wy:.1f})")
        return new_track

    def _cleanup_stale_tracks(self, now: float):
        """Kalıcılık artık KAMERA TARAFINDAN RENK KİLİTLENMİŞ ('locked') track'lere özel
        ve 'sonsuza dek' değil 'çok uzun ama sonlu' (CONFIRMED_TRACK_RETENTION_S, örn.
        5dk): gemi ne kadar uzaklaşırsa uzaklaşsın kilitli bir dubayı unutmaz, ama gürültü
        sonsuza dek birikmez. RENKSİZ track'ler (confidence yüksek olsa — sırf LiDAR'da
        tekrar tekrar görülmüş olsa — bile, kameradan hiç renk almadıysa) kısa süreli
        STALE_TRACK_MAX_AGE_S ile temizlenir — aksi halde dalga/köpük/yanlış küme gibi
        hiç doğrulanmamış şeyler haritayı sonsuza dek şişirirdi (kullanıcı geri bildirimi).
        Son çare olarak MAX_TOTAL_TRACKS aşılırsa en eski görülen (önce renksiz, sonra
        gerekirse renkli) track'ler de silinir."""
        before = len(self.buoy_tracks)
        kept = []
        for t in self.buoy_tracks:
            max_age = CONFIRMED_TRACK_RETENTION_S if t['locked'] else STALE_TRACK_MAX_AGE_S
            if (now - t['last_seen']) > max_age:
                continue
            kept.append(t)

        if len(kept) > MAX_TOTAL_TRACKS:
            # Son çare güvenlik tavanı: önce renksizleri, sonra gerekirse renklileri en
            # eski görülenden başlayarak at (aşırı gürültülü sahne senaryosu).
            kept.sort(key=lambda t: (t['locked'], t['last_seen']))
            dropped_extra = len(kept) - MAX_TOTAL_TRACKS
            self.get_logger().warn(
                f"[TEMİZLİK] Track sayısı MAX_TOTAL_TRACKS({MAX_TOTAL_TRACKS}) aşıldı — "
                f"en eski {dropped_extra} track (öncelik: renksizler) zorla düşürülüyor.")
            kept = kept[dropped_extra:]

        self.buoy_tracks = kept
        removed = before - len(self.buoy_tracks)
        if removed:
            self.get_logger().info(f"[TEMİZLİK] {removed} track hafızadan düşürüldü (yaş/tavan).")

    def reset_map(self):
        """Duba haritasını (buoy_tracks) tamamen temizler — ego-motion/konum (ship_x,
        ship_y, fused_yaw) buna dahil DEĞİL, sadece 'hangi dubalar nerede' bilgisi silinir.
        Klavyeden 'r' ile tetiklenir (bkz. _keypress_listener)."""
        with self._tracks_lock:
            removed = len(self.buoy_tracks)
            self.buoy_tracks = []
            self._next_track_id = 1
        self.get_logger().info(f"🧹 Harita sıfırlandı — {removed} duba track'i temizlendi.")

    def buoy_detection_callback(self, msg: String):
        """camera_vision_node'un `/vision/buoy_detections` üzerinden gönderdiği (x,y,label)
        adaylarını okur ve mevcut renk-oyu füzyonuna besler. Kamera KENDİ BAŞINA ID/konum
        yaratmaz — sadece en yakın LiDAR track'ine bir renk oyu ekler."""
        try:
            data = json.loads(msg.data)
            cam_x, cam_y, label = float(data['x']), float(data['y']), str(data['label'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
            self.get_logger().error(f"[FÜZYON] Geçersiz duba tespiti mesajı: {e}")
            return
        self._fuse_camera_detection(cam_x, cam_y, label)

    def _fuse_camera_detection(self, cam_x: float, cam_y: float, label: str):
        """En yakın LiDAR track'ini bulup oy ekler — AMA en yakın iki aday birbirine çok
        yakınsa (belirsizse) hiçbirine oy vermez. Bu, küçük bir kamera/derinlik kaymasının
        (~30cm) oyu yanlış/komşu bir kümeye 'savurmasını' önler (kullanıcı geri bildirimi)."""
        with self._tracks_lock:
            candidates = [(t['x'], t['y'], t) for t in self.buoy_tracks]
            best_track = find_unambiguous_nearest(
                candidates, cam_x, cam_y, max_distance=FUSION_DISTANCE_M)

            if best_track is None:
                self.get_logger().debug(
                    f"[FÜZYON] '{label}' tespiti hiçbir LiDAR track'ine NET şekilde bağlanamadı "
                    f"(çok uzak veya iki aday birbirine çok yakın/belirsiz) — oy atlandı.")
                return

            self._vote_and_lock_color(best_track, label)

    def _vote_and_lock_color(self, track: dict, label: str):
        if track['locked']:
            if track['color'] != label:
                self.get_logger().info(
                    f"[ÇELİŞKİ] ID {track['id']} kilitli renk={track['color']}, "
                    f"gelen oy={label} yok sayıldı.")
            return

        track['color_votes'][label] += 1
        if track['color_votes'][label] >= COLOR_VOTE_THRESHOLD:
            track['color'] = label
            track['locked'] = True
            self.get_logger().info(
                f"[🔒] ID {track['id']} renk={label} @ ({track['x']:.1f},{track['y']:.1f}) KİLİTLENDİ")

    # -----------------------------------------------------------------------
    # RVIZ2 MARKER VE COSTMAP YAYINLARI
    # -----------------------------------------------------------------------
    def _mapping_circle_points_world(self, segments: int = 36) -> list:
        """Dairesel algılama alanının (MAP_RADIUS_M) sınırını dünya/odom çerçevesinde
        çizmek için nokta listesi üretir. Daire gemi merkezli ama yön-simetrik olduğu
        için fused_yaw'a göre döndürmeye gerek yok."""
        points = []
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            wx = self.ship_x + MAP_RADIUS_M * math.cos(angle)
            wy = self.ship_y + MAP_RADIUS_M * math.sin(angle)
            points.append((wx, wy))
        return points

    def publish_rviz_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = self.ODOM_FRAME

        clr = Marker()
        clr.header.frame_id = frame
        clr.header.stamp = stamp
        clr.action = Marker.DELETEALL
        marker_array.markers.append(clr)

        # --- Robot Çizimi (üçgen — yönelimi kare/küpten daha net ayırt ettirir) ---
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
        robot_m.color.r, robot_m.color.g, robot_m.color.b, robot_m.color.a = 0.0, 0.6, 1.0, 0.9
        # Gövde çerçevesinde (x=ileri+) tek bir üçgen: ileri uç + iki arka köşe.
        robot_m.points = [
            Point(x=0.6, y=0.0, z=0.0),
            Point(x=-0.4, y=0.35, z=0.0),
            Point(x=-0.4, y=-0.35, z=0.0),
        ]
        marker_array.markers.append(robot_m)

        # --- Başlık Oku (geminin baktığı yön — kullanıcı isteği) ---
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

        # --- Algılama Alanı Sınırı (20m yarıçaplı daire, gemiyle birlikte kayar) ---
        circle_m = Marker()
        circle_m.header.frame_id = frame
        circle_m.header.stamp = stamp
        circle_m.ns = 'mapping_area'
        circle_m.id = 2
        circle_m.type = Marker.LINE_STRIP
        circle_m.action = Marker.ADD
        circle_m.scale.x = 0.08
        circle_m.color.r, circle_m.color.g, circle_m.color.b, circle_m.color.a = 0.2, 0.9, 0.9, 0.6
        circle_m.pose.orientation.w = 1.0
        for wx, wy in self._mapping_circle_points_world():
            circle_m.points.append(Point(x=float(wx), y=float(wy), z=0.05))
        marker_array.markers.append(circle_m)

        # --- Başlangıç Konumu İşareti (sabit, node başlangıcındaki (0,0) — test için) ---
        start_m = Marker()
        start_m.header.frame_id = frame
        start_m.header.stamp = stamp
        start_m.ns = 'start_position'
        start_m.id = 0
        start_m.type = Marker.SPHERE
        start_m.action = Marker.ADD
        start_m.pose.position.x, start_m.pose.position.y, start_m.pose.position.z = 0.0, 0.0, 0.15
        start_m.pose.orientation.w = 1.0
        start_m.scale.x = start_m.scale.y = start_m.scale.z = 0.35
        start_m.color.r, start_m.color.g, start_m.color.b, start_m.color.a = 1.0, 0.5, 0.0, 0.9
        marker_array.markers.append(start_m)

        start_txt_m = Marker()
        start_txt_m.header.frame_id = frame
        start_txt_m.header.stamp = stamp
        start_txt_m.ns = 'start_position'
        start_txt_m.id = 1
        start_txt_m.type = Marker.TEXT_VIEW_FACING
        start_txt_m.action = Marker.ADD
        start_txt_m.pose.position.x, start_txt_m.pose.position.y, start_txt_m.pose.position.z = 0.0, 0.0, 0.6
        start_txt_m.pose.orientation.w = 1.0
        start_txt_m.scale.z = 0.3
        start_txt_m.color.r = start_txt_m.color.g = start_txt_m.color.b = start_txt_m.color.a = 1.0
        start_txt_m.text = "START"
        marker_array.markers.append(start_txt_m)

        # --- Duba Çizimi (LiDAR track'leri) ---
        with self._tracks_lock:
            tracks_snapshot = list(self.buoy_tracks)

        cos_y, sin_y = math.cos(-self.fused_yaw), math.sin(-self.fused_yaw)
        kilitli_sayim = defaultdict(int)

        for track in tracks_snapshot:
            renk = (track['color'] or '').lower()
            if track['locked']:
                kilitli_sayim[renk] += 1

            is_raw = track['confidence'] < CONFIRMED_TRACK_MIN_CONFIDENCE

            obj_m = Marker()
            obj_m.header.frame_id = frame
            obj_m.header.stamp = stamp
            obj_m.ns = 'raw_clusters' if is_raw else 'buoys'
            obj_m.id = track['id']
            obj_m.type = Marker.SPHERE
            obj_m.action = Marker.ADD
            obj_m.pose.position.x = float(track['x'])
            obj_m.pose.position.y = float(track['y'])
            obj_m.pose.position.z = 0.15 if is_raw else 0.3
            obj_m.pose.orientation.w = 1.0

            if is_raw:
                obj_m.scale.x = obj_m.scale.y = obj_m.scale.z = 0.15
                obj_m.color.r, obj_m.color.g, obj_m.color.b, obj_m.color.a = 0.6, 0.6, 0.6, 0.35
                marker_array.markers.append(obj_m)
                continue

            conf_ratio = min(float(track['confidence']) / 50.0, 1.0)
            d_scale = 0.3 + (0.3 * conf_ratio)
            obj_m.scale.x, obj_m.scale.y, obj_m.scale.z = d_scale, d_scale, d_scale

            if 'green' in renk: obj_m.color.r, obj_m.color.g, obj_m.color.b = 0.0, 1.0, 0.0
            elif 'red' in renk: obj_m.color.r, obj_m.color.g, obj_m.color.b = 1.0, 0.0, 0.0
            elif 'yellow' in renk: obj_m.color.r, obj_m.color.g, obj_m.color.b = 1.0, 1.0, 0.0
            else: obj_m.color.r, obj_m.color.g, obj_m.color.b = 0.5, 0.5, 0.5  # henüz kilitlenmedi
            obj_m.color.a = 0.95
            marker_array.markers.append(obj_m)

            dx, dy = track['x'] - self.ship_x, track['y'] - self.ship_y
            bx = dx * cos_y - dy * sin_y
            by = dx * sin_y + dy * cos_y
            dist_m = math.hypot(bx, by)
            bearing_deg = math.degrees(math.atan2(by, bx))

            txt_m = Marker()
            txt_m.header.frame_id = frame
            txt_m.header.stamp = stamp
            txt_m.ns = 'buoy_ids'
            txt_m.id = 1000 + track['id']
            txt_m.type = Marker.TEXT_VIEW_FACING
            txt_m.action = Marker.ADD
            txt_m.pose.position.x = float(track['x'])
            txt_m.pose.position.y = float(track['y'])
            txt_m.pose.position.z = 0.7
            txt_m.pose.orientation.w = 1.0
            txt_m.scale.z = 0.3
            txt_m.color.r = txt_m.color.g = txt_m.color.b = txt_m.color.a = 1.0
            kilit = '🔒' if track['locked'] else ''
            txt_m.text = f"#{track['id']} {track['color'] or '?'} {kilit}\n{dist_m:.1f}m {bearing_deg:+.0f}°"
            marker_array.markers.append(txt_m)

        # --- İstatistik Panosu ---
        toplam_kilitli = sum(kilitli_sayim.values())
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
        dashboard.text = (
            f"Konum: ({self.ship_x:.1f}, {self.ship_y:.1f})  Yaw: {math.degrees(self.fused_yaw):.0f}°\n"
            f"Hiz: {self._ego_speed_mps:.2f} m/s | Eslesme: {self._ego_match_quality*100:.0f}% | "
            f"Mesafe: {self._distance_traveled_m:.1f} m\n"
            f"Pusula gecikme: {self._age_str(self._last_odom_wall_time)}\n"
            f"Dubalar: {len(tracks_snapshot)} toplam | {toplam_kilitli} kilitli "
            f"(K:{sum(v for k,v in kilitli_sayim.items() if 'red' in k)} "
            f"Y:{sum(v for k,v in kilitli_sayim.items() if 'green' in k)} "
            f"S:{sum(v for k,v in kilitli_sayim.items() if 'yellow' in k)})\n"
            f"Mapping: {'AKTIF' if self.mapping_active else 'KAPALI'} | "
            f"Algoritma: {'AKTIF' if self.algorithm_active else 'KAPALI'}"
        )
        marker_array.markers.append(dashboard)

        self.marker_pub.publish(marker_array)

    def publish_local_costmap(self):
        grid_cells = int(self.costmap_size / self.costmap_res)
        grid_data = np.zeros((grid_cells, grid_cells), dtype=np.int8)

        with self._tracks_lock:
            tracks_snapshot = list(self.buoy_tracks)

        for track in tracks_snapshot:
            if track['confidence'] < CONFIRMED_TRACK_MIN_CONFIDENCE: continue
            map_x = (track['x'] - self.ship_x) + (self.costmap_size / 2.0)
            map_y = (track['y'] - self.ship_y) + (self.costmap_size / 2.0)
            gx, gy = int(map_x / self.costmap_res), int(map_y / self.costmap_res)

            if 0 <= gx < grid_cells and 0 <= gy < grid_cells:
                renk = (track['color'] or '').lower()
                cost = 100 if 'yellow' in renk else 80
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        nx, ny = gx + i, gy + j
                        if 0 <= nx < grid_cells and 0 <= ny < grid_cells:
                            grid_data[ny, nx] = max(grid_data[ny, nx], cost)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.ODOM_FRAME
        msg.info.resolution, msg.info.width, msg.info.height = float(self.costmap_res), grid_cells, grid_cells
        msg.info.origin.position.x = float(self.ship_x - (self.costmap_size / 2.0))
        msg.info.origin.position.y = float(self.ship_y - (self.costmap_size / 2.0))
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_data.flatten().tolist()
        self.costmap_pub.publish(msg)

    # -----------------------------------------------------------------------
    # GATE-GEÇİŞ NAVİGASYONU + RC-OVERRIDE MOTOR ÇIKIŞI
    # -----------------------------------------------------------------------
    def _publish_neutral_rc(self):
        """Motorları nötr (dur) konuma getirir. /cmd_vel'in aksine, gerçekten tekneyi
        süren /mavros/rc/override'a nötr PWM yazar — kullanılmayan kanallar 65535
        (mavros'un 'bu kanalı görmezden gel' sentineli)."""
        msg = OverrideRCIn()
        msg.channels = [65535] * 18
        msg.channels[KANAL_DUMEN] = calculate_pwm(0.0)
        msg.channels[KANAL_GAZ] = calculate_pwm(0.0)
        self.rc_pub.publish(msg)

    def _apf_timer_callback(self):
        """Sürüş artık kamera callback'ine değil kendi zamanlayıcısına bağlı — motor
        güncelleme hızı YOLO inference süresine bağımlı değil (bkz. dosya başlığı)."""
        if not self.algorithm_active:
            return
        self._gate_drive_hesapla()

    def _gate_drive_hesapla(self):
        """waypoint_generator.py mantığı (Nav2/GPS YOK): A(başlangıç)→B(mission_goal)
        doğrultusunda önde SARI (engel) varsa, o engelin KIRMIZI+YEŞİL (kenar) dubalarına
        göre GENİŞ tarafa bir ara hedef konur (bkz. oak_common.compute_gate_waypoint);
        oraya doğru her tick'te yeniden hesaplanan sürekli bir P-kontrolle (Nav2 action
        YOK) sürülür. Hedef girilmemişse veya hedefe ulaşılmışsa motorlar nötre çekilir."""
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

        with self._tracks_lock:
            tracks_snapshot = list(self.buoy_tracks)

        # Renk şeması: SARI = orta/engel, KIRMIZI+YEŞİL (havuzlanmış) = kenar/gate.
        yellow_buoys, edge_buoys = [], []
        for t in tracks_snapshot:
            if t['confidence'] < CONFIRMED_TRACK_MIN_CONFIDENCE:
                continue
            renk = (t['color'] or '').lower()
            if 'yellow' in renk:
                yellow_buoys.append((t['x'], t['y']))
            elif 'red' in renk or 'green' in renk:
                edge_buoys.append((t['x'], t['y']))

        waypoint = compute_gate_waypoint(
            (self.ship_x, self.ship_y), self.mission_yaw, self.mission_goal,
            yellow_buoys, edge_buoys,
            look_ahead=LOOK_AHEAD_M, waypoint_offset=WAYPOINT_OFFSET_M,
            side_edge_max=SIDE_EDGE_MAX_M, min_safe_dist=MIN_SAFE_DIST_M)
        target = waypoint if waypoint is not None else self.mission_goal

        bearing_to_target = math.atan2(target[1] - self.ship_y, target[0] - self.ship_x)
        heading_error = bearing_to_target - self.fused_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # heading_error > 0 (hedef SOLDA) -> sol motor yavaşlar/sağ hızlanır (waypoint_
        # generator.py'nin referans script'indeki 'a' sola-pivot yönüyle aynı mantık).
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
# MANUEL EVRE KONTROLÜ — KLAVYE DİNLEYİCİ (ayrı thread)
# -----------------------------------------------------------------------
def _keypress_listener(node: 'NjordLidarFusionNode'):
    """Terminalden tek karakter okur (Enter gerekmez): '1' Mapping'i, '2' Algoritmayı
    başlatır/durdurur (toggle), 'h' yerel (x,y) hedef girer, 'r' mapping açıkken
    biriken duba haritasını sıfırlar."""
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
            "[h] Hedef gir (yerel x,y metre)   [r] Haritayı sıfırla   (Ctrl+C: çıkış)")
        while rclpy.ok():
            ch = sys.stdin.read(1)
            if ch == '1':
                node.mapping_active = not node.mapping_active
                node.mapping_active_pub.publish(Bool(data=node.mapping_active))
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
                    node._publish_neutral_rc()  # güvenlik: dur
            elif ch == 'h':
                # waypoint_generator.py / RC-override script'indeki desen: cbreak modunu
                # geçici olarak normale döndürüp input() ile tam satır okuyoruz, sonra
                # cbreak'e geri döneriz.
                node.algorithm_active = False
                node._publish_neutral_rc()  # hedef girilirken güvenlik: dur
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
                    node.get_logger().warn("⚠️  Mapping kapalıyken sıfırlanacak bir harita yok.")
                    continue
                node.reset_map()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    # signal_handler_options=NO: rclpy'nin SIGINT'te context'i KENDİLİĞİNDEN kapatmasını
    # engeller. Aksi halde (varsayılan davranış) SIGINT gelince rclpy context'i bizim
    # 'finally' bloğumuza ulaşmadan ÖNCE geçersiz kılıyor — bu da aşağıdaki GÜVENLİK
    # DURDURMA komutunun (motorları sıfır hıza çekme) sessizce BAŞARISIZ OLMASINA yol
    # açıyordu (gerçek Jetson testinde görüldü: "Failed to publish: publisher's context
    # is invalid"). SignalHandlerOptions.NO ile SIGINT, Python'un normal
    # KeyboardInterrupt'ına dönüşür ve context hâlâ geçerliyken RC override'ı güvenle
    # yayınlayabiliriz.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NjordLidarFusionNode()
    listener = threading.Thread(target=_keypress_listener, args=(node,), daemon=True)
    listener.start()

    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_neutral_rc()  # Güvenlik için motorları durdur
        except Exception as e:
            node.get_logger().error(f"⚠️  GÜVENLİK DURDURMA KOMUTU GÖNDERİLEMEDİ: {e}")
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()  # zaten kapatılmışsa sessizce geçer (exception atmaz)


if __name__ == '__main__':
    main()
