#!/usr/bin/env python3
"""
Njord Füzyon Sistemi v3.4 (FINAL — map frame birleşik sürüm)
============================================================
v3.3 (Odom + drift + kilit) tabanı üzerine v3'ün doğru parçaları taşındı.

TABAN (v3.3'ten korunan):
  - Konum kaynağı  → nav_msgs/Odometry (/mavros/local_position/odom)
  - Landmark tabanlı drift telafisi (EMA + clamp)
  - Yüksek güvende duba konumu KİLİTLEME (locked)
  - map -> velodyne TF broadcaster
  - TÜM frame'ler 'map'

v3'ten TAŞINAN / DÜZELTİLEN:
  - FIX  Bearing işareti REP-103'e göre düzeltildi: ros_angle_rad = -angle_rad
         (kamerada sağ = +offset, ROS'ta sol = +Y). v3.3'teki +angle_rad regresyonu giderildi.
  - ADD  GPS (NavSatFix) → dubaların gerçek lat/lon raporu (buoy_latlon)
  - ADD  Koridor / şerit ortalama → /corridor/offset (Float32)
  - ADD  Periyodik harita özeti logu (log_map_summary)

EK İYİLEŞTİRMELER:
  - Derinlik → yatay menzil düzeltmesi: menzil = z / cos(bearing)
  - detection_buffer sızıntısı temizlendi (nesne silinince buffer key'i de silinir)
  - merge mesafeleri parkura göre inceltildi (same=2.0, any=0.25)
"""

from collections import defaultdict
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy,
    QoSDurabilityPolicy, QoSReliabilityPolicy,
)
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, NavSatFix
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose, Point, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


# ---------------------------------------------------------------------------
# DRIFT TELAFİSİ AYARLARI
# ---------------------------------------------------------------------------
DRIFT_ALPHA    = 0.10   # drift EMA yumuşatması (0.05 = çok yavaş, 0.3 = agresif)
DRIFT_MIN_CONF = 15     # drift ölçümüne katkı için dubanın min. güveni
DRIFT_MAX      = 8.0    # metre; drift tahmini bu sınırın üstüne çıkamaz (emniyet)
LOCK_CONF      = 30     # bu güvenin üstünde duba konumu KİLİTLENİR


# ---------------------------------------------------------------------------
# Yardımcı: LiDAR nokta kümeleme (numpy — Jetson bağımlılık güvenliği için)
# ---------------------------------------------------------------------------

def cluster_lidar_points(points_2d: np.ndarray,
                          cluster_distance: float = 1.5,
                          min_points: int = 2) -> list[dict]:
    if len(points_2d) == 0:
        return []
    used = np.zeros(len(points_2d), dtype=bool)
    clusters = []
    for i in range(len(points_2d)):
        if used[i]:
            continue
        dists = np.linalg.norm(points_2d - points_2d[i], axis=1)
        members = np.where((dists < cluster_distance) & (~used))[0]
        if len(members) < min_points:
            continue
        used[members] = True
        member_pts = points_2d[members]
        center = tuple(np.mean(member_pts, axis=0))
        clusters.append({'center': center, 'size': len(members), 'points': member_pts})
    return clusters


# ---------------------------------------------------------------------------
# Ana node
# ---------------------------------------------------------------------------

class NjordFuzyonSistemi(Node):
    def __init__(self):
        super().__init__('njord_fuzyon_node')
        self.bridge = CvBridge()

        self.get_logger().info("YOLO Modeli yükleniyor...")
        self.model = YOLO('best1JUL.pt')

        self.ship_x, self.ship_y, self.ship_yaw = 0.0, 0.0, 0.0

        # --- GPS (global_position/global) — sadece duba lat/lon raporu için ---
        self.ship_lat = None      # geminin anlık enlemi (derece)
        self.ship_lon = None      # geminin anlık boylamı (derece)
        self.gps_ready = False

        # --- DRIFT TAHMİNİ (landmark tabanlı) ---
        self.drift_x = 0.0
        self.drift_y = 0.0
        self._last_drift_log = 0.0

        # --- TF BROADCASTER ---
        self.tf_bc = TransformBroadcaster(self)

        # --- HAFIZA VE SLAM ---
        self.detected_objects: list[dict] = []
        self.detection_buffer: dict = defaultdict(list)
        self.get_logger().info("🧹 Hafıza sıfırlandı — temiz başlangıç.")
        self.buffer_size       = 15
        self.min_confirmations = 3
        # Parkur için ince merge: bitişik dubalar ayrı kalır
        self.merge_distance_same = 2.0   # aynı etiket → 2.0m içindeyse aynı nesne
        self.merge_distance_any  = 0.25  # farklı etiket → 0.25m içindeyse çakışma
        self.last_cleanup = time.time()

        # --- Periyodik harita özeti log zamanlayıcısı ---
        self.last_map_log = time.time()
        self.map_log_interval = 2.0   # saniye

        self.lidar_clusters: list[dict] = []

        # --- RENK HARİTASI ---
        self.colors = {
            'yellow_buoy': (0, 255, 255),  'yellow buoy': (0, 255, 255), 'yellow': (0, 255, 255),
            'green_buoy':  (0, 255, 0),    'green buoy':  (0, 255, 0),   'green':  (0, 255, 0),
            'red_buoy':    (0, 0, 255),    'red buoy':    (0, 0, 255),   'red':    (0, 0, 255),
            'black_buoy':  (100, 100, 100),'black buoy':  (100, 100, 100),'black': (100, 100, 100),
            'orange_buoy': (0, 165, 255),  'orange buoy': (0, 165, 255), 'orange': (0, 165, 255),
        }

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        obs_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- YAYINCILAR ---
        self.pub_green    = self.create_publisher(PoseArray,   '/vision/green_buoys',  10)
        self.pub_yellow   = self.create_publisher(PoseArray,   '/vision/yellow_buoys', 10)
        self.marker_pub   = self.create_publisher(MarkerArray, '/vision/markers',       10)
        self.yolo_img_pub = self.create_publisher(Image,       '/vision/yolo_output',   10)

        self.pub_pc = self.create_publisher(PointCloud2, '/vision/pointcloud', 10)

        # Nav2 obstacle_layer
        self.obstacle_pub = self.create_publisher(PointCloud2, '/detected_buoys', obs_qos)
        self.last_obstacle_pub = 0.0

        # Waypoint generator
        self.yellow_buoy_pub = self.create_publisher(PointCloud2, '/buoys/yellow', 10)
        self.green_buoy_pub  = self.create_publisher(PointCloud2, '/buoys/green',  10)

        # Confirmed buoys
        self.confirmed_marker_pub = self.create_publisher(
            MarkerArray, '/vision/confirmed_buoys/markers', 10)
        self.confirmed_cloud_pub = self.create_publisher(
            PointCloud2, '/vision/confirmed_buoys/cloud', 10)
        self.confirmed_buoys: list[dict] = []
        self.last_confirmed_pub = 0.0

        # --- ŞERİT ORTALAMA (corridor) ---
        # /corridor/offset (Float32, metre):
        #   +offset → gemi SAĞA kaymış (koridor merkezi solda) → sola düzelt
        #   -offset → gemi SOLA kaymış (koridor merkezi sağda) → sağa düzelt
        #    0.0    → ortada VEYA yeterli duba yok (saf GPS'e devam)
        # NOT: Bu değeri dinleyen node lateral setpoint'i sahiplenmeli;
        #      Nav2 ile aynı anda direksiyonu ele almadığından emin ol.
        self.corridor_offset_pub = self.create_publisher(Float32, '/corridor/offset', 10)
        self.last_corridor_pub = 0.0
        self.corridor_max_dist = 6.0   # m — bundan uzak dubalar koridor sayılmaz
        self.corridor_min_y    = 0.3   # m — gemi hizasındaki çok yakın dubalar atlanır

        # --- MAVLINK / MAVROS BAĞLANTI AYARLARI ---
        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('pose_timeout', 3.0)
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)

        # --- ABONELİKLER ---
        self.pose_ready = False
        self.last_pose_time = 0.0

        # Konum → Odometry (drift telafisi sürekli poz gerektirir)
        self.create_subscription(
            Odometry, odom_topic,
            self.odom_callback, sensor_qos)
        self.get_logger().info(
            f"ℹ️  {odom_topic} dinleniyor (Odom Modu) "
            "(bağlı değilse sistem (0,0,0) ile çalışır).")

        # GPS → dubaların lat/lon'unu hesaplamak için (konumu ETKİLEMEZ)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.gps_callback, sensor_qos)

        self.create_timer(1.0, self.check_mavlink_link)

        # OAK-D kamera sabitleri
        self.base_f_l    = 440.4
        self.base_width  = 640.0
        self.image_width  = 0
        self.image_height = 0
        self.current_f_l  = self.base_f_l

        self.K = np.zeros((3, 3))
        self.camera_info_received = False
        self.create_subscription(
            CameraInfo, '/oak/rgb/camera_info',
            self.camera_info_callback, 10)

        self.latest_depth_frame   = None
        self.latest_velodyne_xyz  = None

        self.get_logger().info("Sensör abonelikleri başlatılıyor...")
        self.create_subscription(Image,       '/oak/rgb/image_raw',    self.rgb_callback,      sensor_qos)
        self.create_subscription(Image,       '/oak/stereo/image_raw', self.depth_callback,    sensor_qos)
        self.create_subscription(PointCloud2, '/velodyne_points',      self.velodyne_callback, sensor_qos)

        self.get_logger().info(
            "🚀 Füzyon + Hafıza + Nav2 Sistemi Aktif! (v3.4 FINAL)\n"
            "   Mesafe kaynağı  → OAK-D stereo depth\n"
            "   Kümeleme kaynağı → /velodyne_points\n"
            "   Konum kaynağı   → Odometry (MAVROS EKF)\n"
            "   Frame           → map (tümü)\n"
            "   /detected_buoys → Nav2 obstacle_layer\n"
            "   /corridor/offset → şerit ortalama\n"
            "   GPS lat/lon      → duba raporu (log + final tablo)"
        )

    # -----------------------------------------------------------------------
    # CALLBACK'LER
    # -----------------------------------------------------------------------

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.K = np.array(msg.k).reshape(3, 3)
            self.camera_info_received = True
            self.get_logger().info("✅ Kamera K Matrisi başarıyla alındı!")

    def depth_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_frame = np.array(cv_img, dtype=np.uint16)
        except Exception as e:
            self.get_logger().error(f"Depth hatası: {e}")

    def velodyne_callback(self, msg: PointCloud2):
        try:
            gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            xyz_list = [[p[0], p[1], p[2]] for p in gen]
            self.latest_velodyne_xyz = np.array(xyz_list, dtype=np.float32) if xyz_list else None
        except Exception as e:
            self.get_logger().error(f"Velodyne hatası: {e}")

    def odom_callback(self, msg: Odometry):
        if not self.pose_ready:
            self.get_logger().info("✅ Odometry alındı, lokal ODOM modu aktif.")
            self.pose_ready = True
        self.last_pose_time = time.time()

        # Odometry'de pozisyon msg.pose.pose altındadır
        self.ship_x = msg.pose.pose.position.x
        self.ship_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.ship_yaw = math.atan2(siny_cosp, cosy_cosp)

        # map -> velodyne TF (drift düzeltilmiş konum)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'velodyne'
        t.transform.translation.x = self.ship_x - self.drift_x
        t.transform.translation.y = self.ship_y - self.drift_y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_bc.sendTransform(t)

    def gps_callback(self, msg: NavSatFix):
        """Geminin gerçek lat/lon'unu al — SADECE duba lat/lon raporu için.
        Konum tahminini etkilemez (o Odometry'den gelir)."""
        if msg.status.status < 0:   # fix yok
            return
        if not self.gps_ready:
            self.get_logger().info(
                f"✅ GPS fix alındı: {msg.latitude:.6f}, {msg.longitude:.6f}")
            self.gps_ready = True
        self.ship_lat = msg.latitude
        self.ship_lon = msg.longitude

    def check_mavlink_link(self):
        if not self.pose_ready:
            return
        if time.time() - self.last_pose_time > self.pose_timeout:
            self.pose_ready = False
            self.get_logger().warn(
                f"⚠️  Odometry verisi {self.pose_timeout:.0f}s'dir gelmiyor — "
                "MAVLink hattı kopmuş veya EKF çökmesi olabilir.")

    def buoy_latlon(self, obj_x: float, obj_y: float):
        """Bir dubanın map (drift düzeltilmiş) konumundan gerçek lat/lon'unu hesaplar.
        map frame ENU: x = Doğu (East), y = Kuzey (North).
        Gemi konumu da drift düzeltilmiş alınır → aradaki bağıl vektör GPS'e eklenir.
        Düz-dünya yaklaşımı (birkaç km için yeterli). GPS yoksa (None, None)."""
        if self.ship_lat is None or self.ship_lon is None:
            return None, None
        gemi_x = self.ship_x - self.drift_x
        gemi_y = self.ship_y - self.drift_y
        east  = obj_x - gemi_x     # Doğu ofseti (m)
        north = obj_y - gemi_y     # Kuzey ofseti (m)
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(self.ship_lat))
        if abs(m_per_deg_lon) < 1e-6:
            return None, None
        buoy_lat = self.ship_lat + (north / m_per_deg_lat)
        buoy_lon = self.ship_lon + (east  / m_per_deg_lon)
        return buoy_lat, buoy_lon

    def get_buoy_color(self, label: str) -> tuple:
        if label in self.colors:
            return self.colors[label]
        norm = label.lower().replace('_', ' ')
        if norm in self.colors:
            return self.colors[norm]
        for kw, val in [('yellow',(0,255,255)),('green',(0,255,0)),
                        ('red',(0,0,255)),('black',(100,100,100)),
                        ('orange',(0,165,255))]:
            if kw in norm:
                return val
        return (180, 180, 180)

    # -----------------------------------------------------------------------
    # DRIFT TELAFİSİ
    # -----------------------------------------------------------------------

    def _drift_guncelle(self, ham_x: float, ham_y: float, obj: dict):
        rez_x = ham_x - obj['x']
        rez_y = ham_y - obj['y']

        if math.hypot(rez_x - self.drift_x, rez_y - self.drift_y) > 5.0:
            return

        self.drift_x = (1 - DRIFT_ALPHA) * self.drift_x + DRIFT_ALPHA * rez_x
        self.drift_y = (1 - DRIFT_ALPHA) * self.drift_y + DRIFT_ALPHA * rez_y

        buyukluk = math.hypot(self.drift_x, self.drift_y)
        if buyukluk > DRIFT_MAX:
            olcek = DRIFT_MAX / buyukluk
            self.drift_x *= olcek
            self.drift_y *= olcek

        now = time.time()
        if now - self._last_drift_log > 5.0:
            self._last_drift_log = now
            self.get_logger().info(
                f"[DRIFT] Tahmin: ({self.drift_x:+.2f}, {self.drift_y:+.2f}) m "
                f"| büyüklük {math.hypot(self.drift_x, self.drift_y):.2f} m")

    # -----------------------------------------------------------------------
    # HAFIZA / SLAM
    # -----------------------------------------------------------------------

    def cleanup_old_objects(self, max_age: float = 30.0):
        current_time = time.time()
        before = len(self.detected_objects)
        # Silinecek nesnelerin buffer key'lerini de topla (sızıntı önleme)
        silinecek_keys = [
            obj['buf_key'] for obj in self.detected_objects
            if obj['confidence'] < 10 and (current_time - obj['last_seen']) > max_age
        ]
        self.detected_objects = [
            obj for obj in self.detected_objects
            if not (obj['confidence'] < 10 and
                    (current_time - obj['last_seen']) > max_age)
        ]
        # Aktif nesnelerin key'leri hariç, ölü buffer'ları temizle
        aktif_keys = {o['buf_key'] for o in self.detected_objects}
        for k in silinecek_keys:
            if k not in aktif_keys and k in self.detection_buffer:
                del self.detection_buffer[k]

        removed = before - len(self.detected_objects)
        if removed:
            self.get_logger().info(f"[TEMİZLİK] {removed} eski nesne silindi.")

    def _find_same_type_object(self, x: float, y: float, label: str):
        closest, min_dist = None, float('inf')
        for obj in self.detected_objects:
            if obj['type'] != label:
                continue
            d = math.hypot(obj['x'] - x, obj['y'] - y)
            if d < min_dist:
                min_dist = d
                closest = obj
        return closest, min_dist

    def update_or_add_object(self, ham_gx: float, ham_gy: float, label: str):
        # Drift düzeltilmiş map konumu
        global_x = ham_gx - self.drift_x
        global_y = ham_gy - self.drift_y

        closest_obj, dist = self._find_same_type_object(global_x, global_y, label)

        if closest_obj is not None and dist < self.merge_distance_same:
            buf_key = closest_obj['buf_key']
        else:
            # Yeni potansiyel nesne: 0.2m ince ızgaraya snap et (bitişik dubalar ayrı key)
            rx = round(global_x / 0.2) * 0.2
            ry = round(global_y / 0.2) * 0.2
            buf_key = f"{label}_{rx:.2f}_{ry:.2f}"

        buf = self.detection_buffer[buf_key]
        buf.append((global_x, global_y))
        if len(buf) > self.buffer_size:
            buf.pop(0)

        if len(buf) < self.min_confirmations:
            return

        positions = np.array(buf)
        med = np.median(positions, axis=0)
        dists_from_med = np.linalg.norm(positions - med, axis=1)
        positions = positions[dists_from_med < 3.0]
        if len(positions) < self.min_confirmations:
            return

        median_x = float(np.median(positions[:, 0]))
        median_y = float(np.median(positions[:, 1]))

        std = np.std(positions, axis=0)
        if float(np.max(std)) > 3.0:
            return

        # --- Yakın AYNI TİP nesne → güncelle (yeni ID yok) ---
        if closest_obj is not None and dist < self.merge_distance_same:
            conf = closest_obj['confidence']

            # Drift ölçümü (yeterince güvenilir dubalardan)
            if conf >= DRIFT_MIN_CONF:
                self._drift_guncelle(ham_gx, ham_gy, closest_obj)

            if conf < LOCK_CONF:
                alpha = max(0.05, 0.4 - conf * 0.007)
                closest_obj['x'] = closest_obj['x'] * (1 - alpha) + median_x * alpha
                closest_obj['y'] = closest_obj['y'] * (1 - alpha) + median_y * alpha
            elif not closest_obj.get('locked'):
                closest_obj['locked'] = True
                self.get_logger().info(
                    f"[🔒] {label:<15s} ({closest_obj['x']:.1f}, "
                    f"{closest_obj['y']:.1f}) KİLİTLENDİ (conf {conf:.0f})")

            closest_obj['confidence'] = min(conf + 1, 50)
            closest_obj['last_seen']  = time.time()
            closest_obj['buf_key']    = buf_key
            return

        # --- Yeni nesne — farklı tip fiziksel çakışma kontrolü ---
        cok_yakin_obj  = None
        cok_yakin_dist = float('inf')
        for obj in self.detected_objects:
            d = math.hypot(obj['x'] - median_x, obj['y'] - median_y)
            if d < cok_yakin_dist:
                cok_yakin_dist = d
                cok_yakin_obj  = obj

        if cok_yakin_dist < self.merge_distance_any and cok_yakin_obj is not None:
            self.get_logger().info(
                f"[~] {label:<15s} ({median_x:.1f}, {median_y:.1f}) "
                f"ES GECİLDİ — {cok_yakin_obj['type']} zaten "
                f"{cok_yakin_dist:.1f}m yakinda")
            return

        new_obj = {
            'x':          median_x,
            'y':          median_y,
            'type':       label,
            'color':      self.get_buoy_color(label),
            'confidence': self.min_confirmations,
            'last_seen':  time.time(),
            'created_at': time.time(),
            'buf_key':    buf_key,
            'locked':     False,
        }
        self.detected_objects.append(new_obj)
        self.get_logger().info(
            f"[+] {label:<15s} ({median_x:.1f}, {median_y:.1f})")

    # -----------------------------------------------------------------------
    # LiDAR KÜMELEME + VİZYON FÜZYONU
    # -----------------------------------------------------------------------

    def update_lidar_clusters(self):
        if self.latest_velodyne_xyz is None or len(self.latest_velodyne_xyz) == 0:
            self.lidar_clusters = []
            return
        pts   = self.latest_velodyne_xyz
        front = pts[(pts[:, 0] > 0) & (pts[:, 0] < 30)]
        if len(front) == 0:
            self.lidar_clusters = []
            return
        cos_y = math.cos(self.ship_yaw)
        sin_y = math.sin(self.ship_yaw)
        # Drift düzeltilmiş map konumu
        gx = (self.ship_x - self.drift_x) + front[:, 0] * cos_y - front[:, 1] * sin_y
        gy = (self.ship_y - self.drift_y) + front[:, 0] * sin_y + front[:, 1] * cos_y
        self.lidar_clusters = cluster_lidar_points(
            np.column_stack((gx, gy)), cluster_distance=1.5, min_points=2)

    def lidar_vision_fusion(self, fusion_distance: float = 2.5):
        self.confirmed_buoys = []
        if not self.lidar_clusters or self.latest_velodyne_xyz is None:
            return
        for cluster in self.lidar_clusters:
            cx, cy = cluster['center']
            closest_obj, min_dist = None, float('inf')
            for obj in self.detected_objects:
                d = math.hypot(obj['x'] - cx, obj['y'] - cy)
                if d < min_dist:
                    min_dist = d
                    closest_obj = obj
            if min_dist < fusion_distance and closest_obj is not None:
                closest_obj['confidence'] = min(closest_obj['confidence'] + 0.5, 50)
                closest_obj['lidar_confirmed'] = True
                if not any(c is closest_obj for c in self.confirmed_buoys):
                    self.confirmed_buoys.append(closest_obj)

    # -----------------------------------------------------------------------
    # KORİDOR / ŞERİT ORTALAMA
    # -----------------------------------------------------------------------

    def publish_corridor_offset(self):
        """Gemiye göre sol ve sağ en yakın dubayı bulur, koridor merkezini hesaplar,
        geminin merkezden lateral sapmasını /corridor/offset ile yayınlar."""
        now = time.time()
        if now - self.last_corridor_pub < 0.2:   # 5 Hz
            return
        self.last_corridor_pub = now

        # Gemi map konumu (drift düzeltilmiş)
        gemi_x = self.ship_x - self.drift_x
        gemi_y = self.ship_y - self.drift_y

        cos_y = math.cos(-self.ship_yaw)
        sin_y = math.sin(-self.ship_yaw)

        left_y  = None   # sol dubanın lokal Y'si (gemiye en yakın sol)
        right_y = None   # sağ dubanın lokal Y'si (gemiye en yakın sağ)
        left_dist  = float('inf')
        right_dist = float('inf')

        for obj in self.detected_objects:
            if obj['confidence'] < 5:
                continue
            # Global(map) → gemi-merkezli lokal frame (X ileri, Y sol)
            dx = obj['x'] - gemi_x
            dy = obj['y'] - gemi_y
            local_x = dx * cos_y - dy * sin_y    # ileri (+)
            local_y = dx * sin_y + dy * cos_y    # sol (+), sağ (-)

            if local_x <= 0 or local_x > self.corridor_max_dist:
                continue
            if abs(local_y) < self.corridor_min_y:
                continue

            d = math.hypot(local_x, local_y)
            if local_y > 0:   # sol taraf
                if d < left_dist:
                    left_dist = d
                    left_y = local_y
            else:             # sağ taraf
                if d < right_dist:
                    right_dist = d
                    right_y = local_y

        offset = 0.0
        if left_y is not None and right_y is not None:
            center_y = (left_y + right_y) / 2.0
            offset = -center_y   # merkez solda(+) ise gemi sağda → -offset
        elif left_y is not None:
            offset = -(left_y - self.corridor_max_dist * 0.4)
            offset = min(offset, 0.0)   # sadece sağa düzeltme
        elif right_y is not None:
            offset = -(right_y + self.corridor_max_dist * 0.4)
            offset = max(offset, 0.0)   # sadece sola düzeltme

        msg = Float32()
        msg.data = float(offset)
        try:
            self.corridor_offset_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'[corridor] {e}')

        if abs(offset) > 0.1:
            ly = f"{left_y:.1f}" if left_y is not None else "-"
            ry = f"{right_y:.1f}" if right_y is not None else "-"
            self.get_logger().info(
                f"[KORİDOR] sol_y:{ly} sağ_y:{ry} → offset:{offset:+.2f}m",
                throttle_duration_sec=1.0)

    # -----------------------------------------------------------------------
    # HARİTA ÖZETİ LOGU
    # -----------------------------------------------------------------------

    def log_map_summary(self):
        now = time.time()
        if now - self.last_map_log < self.map_log_interval:
            return
        self.last_map_log = now
        if not self.detected_objects:
            return

        sorted_objs = sorted(self.detected_objects,
                             key=lambda o: o['confidence'], reverse=True)

        gemi_x = self.ship_x - self.drift_x
        gemi_y = self.ship_y - self.drift_y
        if self.ship_lat is not None:
            head = (f"[HARİTA] {len(self.detected_objects)} nesne | "
                    f"Gemi GPS:({self.ship_lat:.6f},{self.ship_lon:.6f}) "
                    f"Yaw:{math.degrees(self.ship_yaw):.0f}° "
                    f"Drift:({self.drift_x:+.1f},{self.drift_y:+.1f})")
        else:
            head = (f"[HARİTA] {len(self.detected_objects)} nesne | "
                    f"Gemi:({gemi_x:.1f},{gemi_y:.1f}) "
                    f"Yaw:{math.degrees(self.ship_yaw):.0f}°  [GPS yok]")
        lines = [head]

        for i, o in enumerate(sorted_objs, 1):
            lidar = "✓L" if o.get('lidar_confirmed') else "  "
            kilit = "🔒" if o.get('locked') else "  "
            lat, lon = self.buoy_latlon(o['x'], o['y'])
            if lat is not None:
                pos = f"Lat:{lat:.6f} Lon:{lon:.6f}"
            else:
                pos = f"X:{o['x']:6.1f} Y:{o['y']:6.1f}"
            lines.append(
                f"   #{i} {o['type']:<13} {pos}  "
                f"conf:{o['confidence']:4.0f} {lidar} {kilit}"
            )
        self.get_logger().info("\n".join(lines))

    # -----------------------------------------------------------------------
    # YAYINCILAR
    # -----------------------------------------------------------------------

    def publish_confirmed_buoys(self):
        now = time.time()
        if now - self.last_confirmed_pub < 0.3:
            return
        self.last_confirmed_pub = now

        stamp  = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='map')

        pts = [[float(o['x']), float(o['y']), 0.5] for o in self.confirmed_buoys]
        try:
            self.confirmed_cloud_pub.publish(pc2.create_cloud_xyz32(header, pts))
        except Exception as e:
            self.get_logger().error(f'[confirmed_cloud] {e}')

        ma  = MarkerArray()
        clr = Marker()
        clr.header.frame_id = 'map'
        clr.header.stamp    = stamp
        clr.ns      = 'confirmed_buoys'
        clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        for i, obj in enumerate(self.confirmed_buoys):
            bgr = obj.get('color', (180, 180, 180))
            rr, gg, bb = bgr[2]/255.0, bgr[1]/255.0, bgr[0]/255.0

            for offset, (sx, sy, sz, ar, ag, ab, aa) in enumerate([
                (1.0, 1.0, 1.0, rr, gg, bb, 1.0),
                (1.3, 1.3, 0.05, 1.0, 1.0, 1.0, 0.6),
            ]):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp    = stamp
                m.ns      = 'confirmed_buoys'
                m.id      = i * 3 + offset
                m.type    = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(obj['x'])
                m.pose.position.y = float(obj['y'])
                m.pose.position.z = 0.6
                m.pose.orientation.w = 1.0
                m.scale.x = sx; m.scale.y = sy; m.scale.z = sz
                m.color.r = ar; m.color.g = ag; m.color.b = ab; m.color.a = aa
                ma.markers.append(m)

            txt = Marker()
            txt.header.frame_id = 'map'
            txt.header.stamp    = stamp
            txt.ns      = 'confirmed_buoys'
            txt.id      = i * 3 + 2
            txt.type    = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(obj['x'])
            txt.pose.position.y = float(obj['y'])
            txt.pose.position.z = 1.4
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.4
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            kilit = ' 🔒' if obj.get('locked') else ''
            txt.text = f"✓ {obj['type']}{kilit}\nconf: {obj['confidence']:.0f}  LiDAR+Vision"
            ma.markers.append(txt)

        try:
            self.confirmed_marker_pub.publish(ma)
        except Exception as e:
            self.get_logger().error(f'[confirmed_markers] {e}')

    def publish_obstacles(self):
        now = time.time()
        if now - self.last_obstacle_pub < 0.5:
            return
        self.last_obstacle_pub = now

        MIN_CONF = 15
        reliable = [o for o in self.detected_objects if o['confidence'] >= MIN_CONF]
        header   = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')

        if not reliable:
            empty = pc2.create_cloud_xyz32(header, [])
            try:
                self.obstacle_pub.publish(empty)
                self.yellow_buoy_pub.publish(pc2.create_cloud_xyz32(header, []))
                self.green_buoy_pub.publish(pc2.create_cloud_xyz32(header, []))
            except Exception:
                pass
            return

        all_pts = [[float(o['x']), float(o['y']), 0.5] for o in reliable]
        try:
            self.obstacle_pub.publish(pc2.create_cloud_xyz32(header, all_pts))
        except Exception as e:
            self.get_logger().error(f'[obstacle_pub] {e}')
            return

        if not hasattr(self, '_last_obs_count') or self._last_obs_count != len(reliable):
            self._last_obs_count = len(reliable)
            self.get_logger().info(
                f'[Nav2] {len(reliable)} duba /detected_buoys → obstacle_layer')

        yellow_pts = [[float(o['x']), float(o['y']), 0.0]
                      for o in reliable if 'yellow' in o['type'].lower()]
        green_pts  = [[float(o['x']), float(o['y']), 0.0]
                      for o in reliable if 'green'  in o['type'].lower()]

        header2 = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        try:
            self.yellow_buoy_pub.publish(pc2.create_cloud_xyz32(header2, yellow_pts))
            self.green_buoy_pub.publish( pc2.create_cloud_xyz32(header2, green_pts))
        except Exception as e:
            self.get_logger().error(f'[buoy_pub] {e}')

        ny, ng = len(yellow_pts), len(green_pts)
        if getattr(self, '_last_color_count', (-1, -1)) != (ny, ng):
            self._last_color_count = (ny, ng)
            self.get_logger().info(f'[Buoys] Sarı: {ny} | Yeşil: {ng}')

    def publish_rviz_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = 'map'
        mid = 0

        clr = Marker()
        clr.header.frame_id = frame
        clr.header.stamp = stamp
        clr.action = Marker.DELETEALL
        marker_array.markers.append(clr)

        gemi_x = self.ship_x - self.drift_x
        gemi_y = self.ship_y - self.drift_y

        robot_m = Marker()
        robot_m.header.frame_id = frame; robot_m.header.stamp = stamp
        robot_m.ns = 'robot'; robot_m.id = mid; mid += 1
        robot_m.type = Marker.CUBE; robot_m.action = Marker.ADD
        robot_m.pose.position.x = gemi_x
        robot_m.pose.position.y = gemi_y
        robot_m.pose.position.z = 0.20
        robot_m.pose.orientation.x = 0.0
        robot_m.pose.orientation.y = 0.0
        robot_m.pose.orientation.z = math.sin(self.ship_yaw / 2.0)
        robot_m.pose.orientation.w = math.cos(self.ship_yaw / 2.0)
        robot_m.scale.x = 0.8; robot_m.scale.y = 0.4; robot_m.scale.z = 0.15
        robot_m.color.r = 1.0; robot_m.color.g = 0.8
        robot_m.color.b = 0.8; robot_m.color.a = 0.95
        marker_array.markers.append(robot_m)

        arrow_m = Marker()
        arrow_m.header.frame_id = frame; arrow_m.header.stamp = stamp
        arrow_m.ns = 'robot_heading'; arrow_m.id = mid; mid += 1
        arrow_m.type = Marker.ARROW; arrow_m.action = Marker.ADD
        arrow_m.scale.x = 0.15; arrow_m.scale.y = 0.30; arrow_m.scale.z = 0.30
        arrow_m.color.r = 1.0; arrow_m.color.g = 0.1
        arrow_m.color.b = 0.1; arrow_m.color.a = 1.0
        p0 = Point(); p0.x = gemi_x; p0.y = gemi_y; p0.z = 0.35
        p1 = Point()
        p1.x = gemi_x + 3.5 * math.cos(self.ship_yaw)
        p1.y = gemi_y + 3.5 * math.sin(self.ship_yaw)
        p1.z = 0.35
        arrow_m.points = [p0, p1]
        marker_array.markers.append(arrow_m)

        for cluster in self.lidar_clusters:
            cx, cy = cluster['center']
            cl_m = Marker()
            cl_m.header.frame_id = frame; cl_m.header.stamp = stamp
            cl_m.ns = 'lidar_clusters'; cl_m.id = mid; mid += 1
            cl_m.type = Marker.SPHERE; cl_m.action = Marker.ADD
            cl_m.pose.position.x = float(cx); cl_m.pose.position.y = float(cy)
            cl_m.pose.position.z = 0.10; cl_m.pose.orientation.w = 1.0
            size = min(cluster['size'] * 0.04 + 0.12, 0.55)
            cl_m.scale.x = size; cl_m.scale.y = size; cl_m.scale.z = 0.12
            cl_m.color.r = 0.60; cl_m.color.g = 0.90
            cl_m.color.b = 1.00; cl_m.color.a = 0.80
            marker_array.markers.append(cl_m)

        for obj in sorted(self.detected_objects, key=lambda x: x['confidence']):
            if obj['confidence'] < 5:
                continue
            conf_ratio = min(float(obj['confidence']) / 50.0, 1.0)
            bgr = obj.get('color', (180, 180, 180))
            rr, gg, bb = bgr[2]/255.0, bgr[1]/255.0, bgr[0]/255.0

            obj_m = Marker()
            obj_m.header.frame_id = frame; obj_m.header.stamp = stamp
            obj_m.ns = 'vision_objects'; obj_m.id = mid; mid += 1
            obj_m.type = Marker.SPHERE; obj_m.action = Marker.ADD
            obj_m.pose.position.x = float(obj['x']); obj_m.pose.position.y = float(obj['y'])
            obj_m.pose.position.z = 0.50; obj_m.pose.orientation.w = 1.0
            d = 0.30 + 0.35 * conf_ratio
            obj_m.scale.x = d; obj_m.scale.y = d; obj_m.scale.z = 0.30
            obj_m.color.r = rr; obj_m.color.g = gg
            obj_m.color.b = bb; obj_m.color.a = 0.90
            marker_array.markers.append(obj_m)

            if obj['confidence'] > 20:
                txt_m = Marker()
                txt_m.header.frame_id = frame; txt_m.header.stamp = stamp
                txt_m.ns = 'vision_conf_text'; txt_m.id = mid; mid += 1
                txt_m.type = Marker.TEXT_VIEW_FACING; txt_m.action = Marker.ADD
                txt_m.pose.position.x = float(obj['x']); txt_m.pose.position.y = float(obj['y'])
                txt_m.pose.position.z = 0.85; txt_m.pose.orientation.w = 1.0
                txt_m.scale.z = 0.28
                txt_m.color.r = rr; txt_m.color.g = gg
                txt_m.color.b = bb; txt_m.color.a = 1.0
                tag = ' ✓L' if obj.get('lidar_confirmed') else ''
                if obj.get('locked'):
                    tag += ' 🔒'
                txt_m.text = f"{obj['type']} ({obj['confidence']:.0f}){tag}"
                marker_array.markers.append(txt_m)

        high_c = sum(1 for o in self.detected_objects if o['confidence'] > 20)
        med_c  = sum(1 for o in self.detected_objects if 10 <= o['confidence'] <= 20)
        low_c  = sum(1 for o in self.detected_objects if o['confidence'] < 10)
        lock_c = sum(1 for o in self.detected_objects if o.get('locked'))
        info_m = Marker()
        info_m.header.frame_id = frame; info_m.header.stamp = stamp
        info_m.ns = 'system_info'; info_m.id = mid; mid += 1
        info_m.type = Marker.TEXT_VIEW_FACING; info_m.action = Marker.ADD
        info_m.pose.position.x = gemi_x + 6.0
        info_m.pose.position.y = gemi_y + 6.0
        info_m.pose.position.z = 2.0; info_m.pose.orientation.w = 1.0
        info_m.scale.z = 0.35
        info_m.color.r = info_m.color.g = info_m.color.b = info_m.color.a = 1.0
        info_m.text = (
            f"Pos: ({gemi_x:.1f}, {gemi_y:.1f})\n"
            f"Yaw: {math.degrees(self.ship_yaw):.1f} deg\n"
            f"Drift: ({self.drift_x:+.1f}, {self.drift_y:+.1f}) m\n"
            f"Toplam: {len(self.detected_objects)} nesne | Kilitli: {lock_c}\n"
            f"Yuksek(>20): {high_c}  Orta: {med_c}  Dusuk: {low_c}\n"
            f"LiDAR kumeleri: {len(self.lidar_clusters)}"
        )
        marker_array.markers.append(info_m)
        self.marker_pub.publish(marker_array)

    # -----------------------------------------------------------------------
    # ANA SENKRON CALLBACK
    # -----------------------------------------------------------------------

    def rgb_callback(self, image_msg: Image):
        if not hasattr(self, '_rgb_count'):
            self._rgb_count = 0
            self.get_logger().info("✅ rgb_callback ilk kez tetiklendi!")
        self._rgb_count += 1
        if self._rgb_count % 30 == 0:
            self.get_logger().info(f"[RGB] {self._rgb_count} frame işlendi.")

        if not self.pose_ready:
            self.get_logger().warn(
                "Odometry verisi yok — konum (0,0,0) ile çalışılıyor.",
                throttle_duration_sec=10.0)

        depth_available = self.latest_depth_frame is not None

        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB dönüşüm hatası: {e}")
            return

        rgb_h, rgb_w = cv_rgb.shape[:2]

        if self.image_width != rgb_w:
            self.image_width  = rgb_w
            self.image_height = rgb_h
            self.current_f_l  = self.base_f_l * (rgb_w / self.base_width)
            self.get_logger().info(
                f"Görüntü boyutu: {rgb_w}x{rgb_h}, "
                f"focal_length: {self.current_f_l:.1f}px")

        if depth_available:
            depth_h, depth_w = self.latest_depth_frame.shape[:2]
            scale_x = depth_w / rgb_w
            scale_y = depth_h / rgb_h

        self.update_lidar_clusters()
        self.lidar_vision_fusion()

        results = self.model.predict(cv_rgb, conf=0.5, verbose=False)
        boxes   = results[0].boxes

        now_stamp  = self.get_clock().now().to_msg()
        green_msg  = PoseArray()
        green_msg.header.frame_id = 'map'
        green_msg.header.stamp    = now_stamp
        yellow_msg = PoseArray()
        yellow_msg.header = green_msg.header

        self._frame_pc_points = []

        for r in boxes:
            x1, y1, x2, y2 = map(int, r.xyxy[0])
            label = self.model.names[int(r.cls[0])]
            color = self.get_buoy_color(label)

            u_c = (x1 + x2) // 2
            v_c = (y1 + y2) // 2

            # pixel_offset: nesne sağda → +, solda → -
            pixel_offset = u_c - (rgb_w / 2.0)
            angle_rad    = math.atan(pixel_offset / self.current_f_l)
            angle_deg    = math.degrees(angle_rad)   # sadece log için (kamera açısı)

            # REP-103: kamerada sağ = +offset ama ROS'ta sol = +Y → açıyı ters çevir
            ros_angle_rad = -angle_rad

            if depth_available:
                u_d = int(u_c * scale_x)
                v_d = int(v_c * scale_y)
                roi = self.latest_depth_frame[
                    max(0, v_d-10):min(depth_h, v_d+10),
                    max(0, u_d-10):min(depth_w, u_d+10)]
                valid_px = roi[(roi > 100) & (roi < 10000)]

                if len(valid_px) > 0:
                    z = float(np.min(valid_px)) / 1000.0   # mm → m (ileri derinlik)

                    # Derinlik → yatay menzil düzeltmesi
                    menzil = z / max(math.cos(ros_angle_rad), 1e-3)

                    self.get_logger().info(
                        f"TESPİT: {label} | Mesafe: {z:.2f}m | Açı: {angle_deg:.1f}°",
                        throttle_duration_sec=0.5)

                    x_m = z * math.tan(ros_angle_rad)
                    y_m = (v_c - rgb_h / 2.0) * z / self.current_f_l

                    total_angle = self.ship_yaw + ros_angle_rad
                    buoy_gx = self.ship_x + menzil * math.cos(total_angle)
                    buoy_gy = self.ship_y + menzil * math.sin(total_angle)

                    self.update_or_add_object(buoy_gx, buoy_gy, label)
                    self._frame_pc_points.append([z, -x_m, -y_m])
                    text = f"{label}: {z:.2f}m | {angle_deg:.1f}°"
                else:
                    text = f"{label}: depth yok"
            else:
                text = f"{label}: depth bekleniyor"

            cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), color, 2)
            cv2.putText(cv_rgb, text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if self._frame_pc_points:
            h = Header(stamp=now_stamp, frame_id='oak_rgb_camera_optical_frame')
            try:
                self.pub_pc.publish(pc2.create_cloud_xyz32(h, self._frame_pc_points))
            except Exception as e:
                self.get_logger().error(f"PC2 yayın hatası: {e}")

        if time.time() - self.last_cleanup > 30:
            self.cleanup_old_objects()
            self.last_cleanup = time.time()

        MIN_CONFIDENCE = 10
        for obj in self.detected_objects:
            if obj['confidence'] >= MIN_CONFIDENCE:
                pose = Pose()
                pose.position.x = float(obj['x'])
                pose.position.y = float(obj['y'])
                if 'green' in obj['type'].lower():
                    green_msg.poses.append(pose)
                elif 'yellow' in obj['type'].lower():
                    yellow_msg.poses.append(pose)

        self.pub_green.publish(green_msg)
        self.pub_yellow.publish(yellow_msg)

        self.publish_rviz_markers()
        self.publish_obstacles()
        self.publish_confirmed_buoys()
        self.publish_corridor_offset()
        self.log_map_summary()

        try:
            yolo_ros_msg = self.bridge.cv2_to_imgmsg(cv_rgb, 'bgr8')
            yolo_ros_msg.header = image_msg.header
            self.yolo_img_pub.publish(yolo_ros_msg)
        except Exception as e:
            self.get_logger().error(f"Görüntü yayın hatası: {e}")


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

def _print_final_report(node: 'NjordFuzyonSistemi'):
    objs = node.detected_objects
    if not objs:
        print("\n[Rapor] Hiç nesne kaydedilmedi.")
        return

    sorted_objs = sorted(objs, key=lambda o: o['confidence'], reverse=True)
    current_time = time.time()

    print("\n" + "=" * 100)
    print(f"{'FİNAL HARİTA RAPORU':^100}")
    print("=" * 100)
    print(f"{'#':<4} {'Tip':<14} {'Lat':>12} {'Lon':>12} "
          f"{'X':>7} {'Y':>7} {'Güven':>7} {'LiDAR':>6} {'Kilit':>6} {'Yaş(s)':>8}")
    print("-" * 100)
    for i, obj in enumerate(sorted_objs, 1):
        age    = current_time - obj.get('created_at', current_time)
        lidar  = "✓" if obj.get('lidar_confirmed') else "-"
        kilit  = "🔒" if obj.get('locked') else "-"
        lat, lon = node.buoy_latlon(obj['x'], obj['y'])
        lat_s = f"{lat:.6f}" if lat is not None else "   -    "
        lon_s = f"{lon:.6f}" if lon is not None else "   -    "
        print(f"{i:<4} {obj['type']:<14} {lat_s:>12} {lon_s:>12} "
              f"{obj['x']:>7.1f} {obj['y']:>7.1f} "
              f"{obj['confidence']:>7.0f} {lidar:>6} {kilit:>6} {age:>8.1f}")
    print("=" * 100)
    print(f"Toplam          : {len(objs)} nesne")
    print(f"Yüksek (>20)    : {sum(1 for o in objs if o['confidence'] > 20)}")
    print(f"Orta   (10-20)  : {sum(1 for o in objs if 10 <= o['confidence'] <= 20)}")
    print(f"Düşük  (<10)    : {sum(1 for o in objs if o['confidence'] < 10)}")
    print(f"LiDAR onaylı    : {sum(1 for o in objs if o.get('lidar_confirmed'))}")
    print(f"Kilitli         : {sum(1 for o in objs if o.get('locked'))}")
    print(f"Son drift       : ({node.drift_x:+.2f}, {node.drift_y:+.2f}) m")
    print("=" * 100 + "\n")


def main():
    rclpy.init()
    node = NjordFuzyonSistemi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _print_final_report(node)
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
