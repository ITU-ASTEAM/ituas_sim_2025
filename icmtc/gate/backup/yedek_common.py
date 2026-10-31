#!/usr/bin/env python3
"""
Njord YEDEK Sistemi — Ortak Sabitler ve Saf Fonksiyonlar
`yedek_lidar_node.py` tarafından import edilir. Bu modül ROS'suzdur (rclpy'a bağımlı
değildir) — bağımsız test edilebilir.

BU MODÜL, primary/oak_common.py'YE HİÇBİR ŞEKİLDE BAĞIMLI DEĞİLDİR (import etmez, kopyalamaz
çalışma zamanında) — AS klasörü silinse/bozulsa bile YEDEK çalışmaya devam eder. Bazı
fonksiyonlar (LiDAR ön işleme, ego-motion, PWM/mixing) primary/oak_common.py'dekiyle aynı
matematiği içerir çünkü bu fonksiyonlar zaten kameradan bağımsızdır — bilinçli bir kod
kopyası, bir bağımlılık değil.
"""

import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

# ---------------------------------------------------------------------------
# AYARLANABİLİR SABİTLER — yedek_config.json'dan yüklenir (kod değiştirmeden, tek
# dosyayı düzenleyip node'u yeniden başlatarak tüm ayarlar değiştirilebilir).
# ---------------------------------------------------------------------------

def _load_config() -> dict:
    """yedek_config.json'u okur. Dosya JSONC'dir (satır sonu '//' yorumları içerir) —
    standart json modülü yorumu desteklemediği için her satırda '//' ve sonrasını
    parse etmeden ÖNCE atıyoruz (primary/oak_common._load_config ile aynı yaklaşım). Bu
    projede config sadece sayısal değerler içerdiği için naif satır-bazlı '//'
    temizleme güvenlidir."""
    config_path = Path(__file__).resolve().parent / 'yedek_config.json'
    text = config_path.read_text(encoding='utf-8')
    stripped_lines = [line[:line.find('//')] if '//' in line else line
                       for line in text.splitlines()]
    try:
        return json.loads('\n'.join(stripped_lines))
    except json.JSONDecodeError as e:
        raise RuntimeError(f"yedek_config.json parse edilemedi: {e}") from e


_CFG = _load_config()

# --- LiDAR ön işleme ---
LIDAR_Z_MIN = _CFG['LIDAR_Z_MIN']
LIDAR_Z_MAX = _CFG['LIDAR_Z_MAX']
# DİKKAT: bu değer primary/njord_config.json'daki LIDAR_CLUSTER_DISTANCE'tan BİLEREK
# FARKLI (küçük) — bkz. yedek_config.json'daki ayrıntılı açıklama: GATE_MIN_WIDTH_M'den
# büyük olursa iki gate dubası tek kümede birleşip çift hiç oluşmaz.
LIDAR_CLUSTER_DISTANCE = _CFG['LIDAR_CLUSTER_DISTANCE']
LIDAR_MIN_CLUSTER_POINTS = _CFG['LIDAR_MIN_CLUSTER_POINTS']
MAX_LIDAR_POINTS_PER_SCAN = _CFG['MAX_LIDAR_POINTS_PER_SCAN']

# Gemi merkezli DAİRESEL algılama/okuma yarıçapı ("belirli bir mesafe") — hem hangi
# LiDAR noktalarının kümelemeye dahil edileceğini SINIRLAR hem de gate-çifti aramasının
# lookahead sınırıdır (bkz. find_gate_candidates). AS'daki MAP_RADIUS_M'den (20m, tüm
# haritalama alanı) kasıtlı olarak KÜÇÜK — YEDEK sadece hemen önündeki tek gate'le
# ilgilenir, uzak/alakasız kümelerle vakit kaybetmez.
YEDEK_DETECTION_RADIUS_M = _CFG['YEDEK_DETECTION_RADIUS_M']

# --- LiDAR Ego-Motion (GPS'siz konum kestirimi) — AS ile aynı anlam/mantık ---
EGO_MATCH_RADIUS_M = _CFG['EGO_MATCH_RADIUS_M']
MIN_EGO_MATCHES = _CFG['MIN_EGO_MATCHES']
MAX_SHIP_SPEED_MPS = _CFG['MAX_SHIP_SPEED_MPS']
MAX_SAFE_YAW_RATE_DEG_S = _CFG['MAX_SAFE_YAW_RATE_DEG_S']

# --- Kamerasız Gate-Çifti Tespiti ---
# gorev.jpg (9.2 Entering the Strait): gate genişliği 1.5-3 m.
GATE_MIN_WIDTH_M = _CFG['GATE_MIN_WIDTH_M']
GATE_MAX_WIDTH_M = _CFG['GATE_MAX_WIDTH_M']
GATE_WIDTH_MARGIN_M = _CFG['GATE_WIDTH_MARGIN_M']
GATE_PAIR_DEPTH_TOLERANCE_M = _CFG['GATE_PAIR_DEPTH_TOLERANCE_M']
GATE_CONFIRM_COUNT = _CFG['GATE_CONFIRM_COUNT']
GATE_MATCH_RADIUS_M = _CFG['GATE_MATCH_RADIUS_M']
WAYPOINT_OFFSET_M = _CFG['WAYPOINT_OFFSET_M']
GOAL_TOLERANCE_M = _CFG['GOAL_TOLERANCE_M']

# --- Yönelim (yaw) kaynak seçimi: OAK IMU + Pixhawk füzyon / tek kaynağa düşme ---
IMU_STALE_TIMEOUT_S = _CFG['IMU_STALE_TIMEOUT_S']
ODOM_STALE_TIMEOUT_S = _CFG['ODOM_STALE_TIMEOUT_S']

# --- Sürüş P-Kontrolü ---
STEER_KP = _CFG['STEER_KP']
MAX_STEER_AXIS = _CFG['MAX_STEER_AXIS']
MIN_THROTTLE_AXIS = _CFG['MIN_THROTTLE_AXIS']

# --- Motor Çıkışı (RC Override) ---
KANAL_DUMEN = _CFG['KANAL_DUMEN']
KANAL_GAZ = _CFG['KANAL_GAZ']
DUMEN_YONU = _CFG['DUMEN_YONU']
RC_PWM_CENTER = _CFG['RC_PWM_CENTER']
RC_PWM_SCALE = _CFG['RC_PWM_SCALE']
RC_PWM_DEADZONE = _CFG['RC_PWM_DEADZONE']
RC_PWM_MIN = _CFG['RC_PWM_MIN']
RC_PWM_MAX = _CFG['RC_PWM_MAX']


# ---------------------------------------------------------------------------
# SAF FONKSİYONLAR — LiDAR ön işleme, kümeleme, ego-motion
# (primary/oak_common.py ile aynı matematik — bkz. modül başlığı)
# ---------------------------------------------------------------------------

def point_in_range(x_body: float, y_body: float, radius: float = YEDEK_DETECTION_RADIUS_M) -> bool:
    """Gövde çerçevesindeki bir noktanın gemi merkezli dairesel algılama alanı
    (yarıçap=radius) içinde olup olmadığını test eder."""
    return math.hypot(x_body, y_body) <= radius


def project_lidar_to_2d(xyz_points: np.ndarray,
                         z_min: float = LIDAR_Z_MIN,
                         z_max: float = LIDAR_Z_MAX,
                         radius: float = YEDEK_DETECTION_RADIUS_M) -> np.ndarray:
    """3D Velodyne bulutunu z-bandı + 2D izdüşüm + dairesel algılama-alanı filtresiyle
    'sanal 2D LiDAR'a indirger."""
    if xyz_points is None or len(xyz_points) == 0:
        return np.empty((0, 2), dtype=np.float32)
    z = xyz_points[:, 2]
    banded = xyz_points[(z >= z_min) & (z <= z_max)]
    if len(banded) == 0:
        return np.empty((0, 2), dtype=np.float32)
    xy = banded[:, :2]
    dist = np.linalg.norm(xy, axis=1)
    return xy[dist <= radius]


def cap_point_count(points_2d: np.ndarray, max_points: int = MAX_LIDAR_POINTS_PER_SCAN,
                     rng: np.random.Generator = None) -> np.ndarray:
    """Bir taramadaki nokta sayısı max_points'i aşarsa rastgele altörnekleme yapar —
    işlem süresini sabit bir üst sınırda tutmak için son çare bir güvenlik supabı."""
    n = len(points_2d)
    if n <= max_points:
        return points_2d
    rng = rng or np.random.default_rng()
    idx = rng.choice(n, size=max_points, replace=False)
    return points_2d[idx]


def cluster_lidar_points(points_2d: np.ndarray,
                          cluster_distance: float = LIDAR_CLUSTER_DISTANCE,
                          min_points: int = LIDAR_MIN_CLUSTER_POINTS) -> list:
    """Mesafe-eşikli, tohum-merkezli açgözlü kümeleme (primary/oak_common.cluster_lidar_points
    ile birebir aynı algoritma): sırayla ilk kullanılmamış noktayı "tohum" alıp ondan
    `cluster_distance` içindeki tüm kullanılmamış noktaları TEK SEFERDE o kümeye toplar.
    Komşu arama `cKDTree.query_ball_point` ile hızlandırılmıştır (O(n log n)'e yakın)."""
    n = len(points_2d)
    if n == 0:
        return []

    tree = cKDTree(points_2d)
    used = np.zeros(n, dtype=bool)
    clusters = []
    for i in range(n):
        if used[i]:
            continue
        candidate_idx = tree.query_ball_point(points_2d[i], r=cluster_distance)
        members = [j for j in candidate_idx
                   if not used[j] and np.linalg.norm(points_2d[j] - points_2d[i]) < cluster_distance]
        if len(members) < min_points:
            continue
        used[members] = True
        member_pts = points_2d[members]
        center = tuple(np.mean(member_pts, axis=0))
        clusters.append({'center': center, 'size': len(members)})
    return clusters


def rotate_xy(x: float, y: float, angle: float) -> tuple:
    c, s = math.cos(angle), math.sin(angle)
    return x * c - y * s, x * s + y * c


def estimate_translation(prev_centers: list, curr_centers: list, dyaw: float,
                          match_radius: float = EGO_MATCH_RADIUS_M) -> tuple:
    """Rotasyon (dyaw) bilindiği için sadece öteleme çözülür (primary/oak_common ile aynı
    mantık): önceki karenin kümeleri -dyaw kadar döndürülüp şimdiki kümelerle
    en-yakın-komşu eşleştirilir; eşleşen çiftlerin medyan kayması gövde-çerçevesinde
    (dx_body, dy_body) döndürülür. 3. değer eşleşme sayısıdır (güven göstergesi)."""
    if not prev_centers or not curr_centers:
        return 0.0, 0.0, 0

    rotated_prev = [rotate_xy(x, y, -dyaw) for (x, y) in prev_centers]
    used_curr = set()
    deltas = []
    for rx, ry in rotated_prev:
        best_j, best_d = None, match_radius
        for j, (cx, cy) in enumerate(curr_centers):
            if j in used_curr:
                continue
            d = math.hypot(rx - cx, ry - cy)
            if d < best_d:
                best_d, best_j = d, j
        if best_j is not None:
            used_curr.add(best_j)
            cx, cy = curr_centers[best_j]
            deltas.append((rx - cx, ry - cy))

    if not deltas:
        return 0.0, 0.0, 0
    deltas = np.array(deltas)
    dx_body, dy_body = np.median(deltas, axis=0)
    return float(dx_body), float(dy_body), len(deltas)


# ---------------------------------------------------------------------------
# KAMERASIZ GATE-ÇİFTİ TESPİTİ — SAF (bu modülün asıl yeni katkısı)
# ---------------------------------------------------------------------------

def is_in_front_of(px: float, py: float, rx: float, ry: float, ref_yaw: float) -> bool:
    """(px,py) noktası (rx,ry)'den ref_yaw yönüne göre ÖNDE mi?"""
    dx, dy = px - rx, py - ry
    return (dx * math.cos(ref_yaw) + dy * math.sin(ref_yaw)) > 0


def find_gate_candidates(clusters_world: list, ship_xy: tuple, ref_yaw: float,
                          detection_radius: float = YEDEK_DETECTION_RADIUS_M,
                          min_width: float = GATE_MIN_WIDTH_M,
                          max_width: float = GATE_MAX_WIDTH_M,
                          width_margin: float = GATE_WIDTH_MARGIN_M,
                          depth_tolerance: float = GATE_PAIR_DEPTH_TOLERANCE_M) -> list:
    """Kamera/renk YOKKEN, LiDAR kümeleri arasında "bu ikisi bir gate mi?" testini
    SAF GEOMETRİYLE yapar. clusters_world: [(x,y), ...] (dünya/odom çerçevesi).

    Bir çift (a,b) şu şartların HEPSİNİ sağlarsa aday sayılır:
      1) ikisi de detection_radius içinde VE geminin önünde (is_in_front_of),
      2) aralarındaki mesafe [min_width-width_margin, max_width+width_margin]
         aralığında (gorev.jpg gate genişliği + ölçüm-gürültüsü payı),
      3) gemiden mesafeleri birbirinden depth_tolerance'tan fazla farklı DEĞİL
         (aksi halde farklı derinlikteki iki alakasız duba yanlışlıkla eşleşir).

    Döner: [(mid_x, mid_y, dist_to_ship, buoy_a, buoy_b), ...] — GEMİYE EN YAKINDAN
    UZAĞA sıralı (en yakın aday çağıran tarafından seçilmeli)."""
    rx, ry = ship_xy
    nearby = [
        (cx, cy) for (cx, cy) in clusters_world
        if math.hypot(cx - rx, cy - ry) <= detection_radius
        and is_in_front_of(cx, cy, rx, ry, ref_yaw)
    ]

    lo, hi = min_width - width_margin, max_width + width_margin
    candidates = []
    n = len(nearby)
    for i in range(n):
        ax, ay = nearby[i]
        dist_a = math.hypot(ax - rx, ay - ry)
        for j in range(i + 1, n):
            bx, by = nearby[j]
            width = math.hypot(bx - ax, by - ay)
            if not (lo <= width <= hi):
                continue
            dist_b = math.hypot(bx - rx, by - ry)
            if abs(dist_a - dist_b) > depth_tolerance:
                continue
            mid_x, mid_y = (ax + bx) / 2.0, (ay + by) / 2.0
            dist_mid = math.hypot(mid_x - rx, mid_y - ry)
            candidates.append((mid_x, mid_y, dist_mid, (ax, ay), (bx, by)))

    candidates.sort(key=lambda c: c[2])
    return candidates


def classify_gate_sides(buoy_a: tuple, buoy_b: tuple, ship_xy: tuple, ref_yaw: float) -> tuple:
    """SADECE loglama/telemetri için: iki dubadan hangisi geminin o anki yönüne göre
    SAĞDA (yeşil/sancak VARSAYIMI) hangisi SOLDA (kırmızı/iskele VARSAYIMI) etiketler.
    GERÇEK KAMERA DOĞRULAMASI YOK — direksiyon matematiği bu etikete bağlı değildir
    (bkz. compute_yedek_gate_waypoint, sadece orta noktayı kullanır).
    Döner: (left_xy, right_xy)."""
    rx, ry = ship_xy
    right_x, right_y = math.sin(ref_yaw), -math.cos(ref_yaw)

    def side_dot(b):
        dx, dy = b[0] - rx, b[1] - ry
        return dx * right_x + dy * right_y

    if side_dot(buoy_a) > side_dot(buoy_b):
        return buoy_b, buoy_a  # (left, right)
    return buoy_a, buoy_b


def compute_yedek_gate_waypoint(ship_xy: tuple, ref_yaw: float,
                                 gate_candidate, waypoint_offset: float = WAYPOINT_OFFSET_M):
    """primary/oak_common.compute_gate_waypoint ile AYNI SÖZLEŞME: onaylı bir gate_candidate
    varsa (mid_x, mid_y, ...) orta noktasına waypoint_offset kadar ileri bir ara hedef
    döner; gate_candidate None ise None döner (çağıran doğrudan goal_xy'ye —yani
    pusula/operatör hedefine— yönelmeli)."""
    if gate_candidate is None:
        return None
    mid_x, mid_y = gate_candidate[0], gate_candidate[1]
    wx = mid_x + math.cos(ref_yaw) * waypoint_offset
    wy = mid_y + math.sin(ref_yaw) * waypoint_offset
    return (wx, wy)


# ---------------------------------------------------------------------------
# MOTOR ÇIKIŞI (RC Override mixing/PWM) — SAF, primary/oak_common ile aynı matematik
# ---------------------------------------------------------------------------

def calculate_pwm(axis_val: float, center: int = RC_PWM_CENTER, scale: int = RC_PWM_SCALE,
                   deadzone: int = RC_PWM_DEADZONE, pwm_min: int = RC_PWM_MIN,
                   pwm_max: int = RC_PWM_MAX) -> int:
    """[-1,1] civarı bir eksen değerini PWM'e çevirir, merkeze yakınsa tam nötr döner,
    sonucu [pwm_min, pwm_max] aralığına kırpar."""
    val = center + int(axis_val * scale)
    if abs(val - center) < deadzone:
        return center
    return max(pwm_min, min(pwm_max, val))


def mix_gaz_dumen(sol: float, sag: float, dumen_yonu: int = DUMEN_YONU) -> tuple:
    """İstenen sol/sağ motor değerlerinden (ArduPilot'un skid-steer mixer'inin
    CH1/CH3'ten beklediği) gaz/dümen eksenlerini üretir. Döner: (gaz, dumen)."""
    gaz = (sol + sag) / 2.0
    dumen = ((sol - sag) / 2.0) * dumen_yonu
    return gaz, dumen
