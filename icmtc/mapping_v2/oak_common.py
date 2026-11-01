#!/usr/bin/env python3
"""
Njord Füzyon Sistemi — Ortak Sabitler ve Saf Fonksiyonlar
Hem `lidar_fusion_node.py` hem de `camera_vision_node.py` tarafından import edilir.
Bu modül ROS'suzdur (rclpy'a bağımlı değildir) — bağımsız test edilebilir.
"""

import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

# ---------------------------------------------------------------------------
# AYARLANABİLİR SABİTLER — njord_config.json'dan yüklenir (kod değiştirmeden,
# tek dosyayı düzenleyip node'ları yeniden başlatarak tüm ayarlar değiştirilebilir).
# ---------------------------------------------------------------------------

def _load_config() -> dict:
    """njord_config.json'u okur. Dosya JSONC'dir (satır sonu '//' yorumları içerir) —
    standart json modülü yorumu desteklemediği için her satırda '//' ve sonrasını
    parse etmeden ÖNCE atıyoruz. Bu projede config sadece sayısal değerler içerdiği
    (metin/URL değeri yok) için naif satır-bazlı '//' temizleme güvenlidir."""
    config_path = Path(__file__).resolve().parent / 'njord_config.json'
    text = config_path.read_text(encoding='utf-8')
    stripped_lines = [line[:line.find('//')] if '//' in line else line
                       for line in text.splitlines()]
    try:
        return json.loads('\n'.join(stripped_lines))
    except json.JSONDecodeError as e:
        raise RuntimeError(f"njord_config.json parse edilemedi: {e}") from e


_CFG = _load_config()

# --- LiDAR ön işleme ---
LIDAR_Z_MIN = _CFG['LIDAR_Z_MIN']
LIDAR_Z_MAX = _CFG['LIDAR_Z_MAX']
LIDAR_CLUSTER_DISTANCE = _CFG['LIDAR_CLUSTER_DISTANCE']
LIDAR_MIN_CLUSTER_POINTS = _CFG['LIDAR_MIN_CLUSTER_POINTS']
MAX_LIDAR_POINTS_PER_SCAN = _CFG['MAX_LIDAR_POINTS_PER_SCAN']

# Algılama/haritalama alanı — gemi merkezli DAİRESEL yarıçap (yönlü kutu yerine). Bu
# alan sadece hangi LiDAR noktalarının kümelemeye dahil edileceğini belirler; bir track
# ONAYLANDIKTAN sonra bu alanın dışına çıkması onu SİLMEZ (bkz. CONFIRMED_TRACK_MIN_CONFIDENCE).
MAP_RADIUS_M = _CFG['MAP_RADIUS_M']

EGO_MATCH_RADIUS_M = _CFG['EGO_MATCH_RADIUS_M']
MIN_EGO_MATCHES = _CFG['MIN_EGO_MATCHES']
MAX_SHIP_SPEED_MPS = _CFG['MAX_SHIP_SPEED_MPS']
# Gemi bundan hızlı dönüyorsa fused_yaw'ın o anki değeri güvenilmez sayılır — var olan
# duba track'lerinin konumu bu tur sırasında GÜNCELLENMEZ (bkz. lidar_fusion_node
# ._update_buoy_tracks). "Arkada kalan duba birden yanımıza savruluyor" sorununun
# birincil çözümü.
MAX_SAFE_YAW_RATE_DEG_S = _CFG['MAX_SAFE_YAW_RATE_DEG_S']

TRACK_MATCH_RADIUS_M = _CFG['TRACK_MATCH_RADIUS_M']
# RENGİ KİLİTLENMEMİŞ (kamera onayı olmayan) track'ler bu süre görülmezse silinir —
# confidence yüksek olsa (sırf LiDAR'da tekrar görülmüş olsa) bile geçerlidir; aksi
# halde gürültü (dalga/köpük/yanlış küme) haritayı sonsuza dek şişirir.
STALE_TRACK_MAX_AGE_S = _CFG['STALE_TRACK_MAX_AGE_S']
MAX_TRACK_CONFIDENCE = _CFG['MAX_TRACK_CONFIDENCE']
# SADECE RViz ham/tekrar-görülmüş ayrımı ve APF/costmap'in bir track'i dikkate almaya
# değer sayması için kullanılır — KALICILIK ANLAMINA GELMEZ (bkz. CONFIRMED_TRACK_RETENTION_S).
CONFIRMED_TRACK_MIN_CONFIDENCE = _CFG['CONFIRMED_TRACK_MIN_CONFIDENCE']
# RENGİ KİLİTLENMİŞ (kamera tarafından gerçekten onaylanmış) bir track bu süre
# görülmezse silinir — "sonsuza dek" değil "çok uzun ama sonlu" kalıcılık.
CONFIRMED_TRACK_RETENTION_S = _CFG['CONFIRMED_TRACK_RETENTION_S']
# Son çare güvenlik tavanı: yukarıdakiler yetersiz kalırsa (aşırı gürültü) bu sayı
# aşılınca en eski görülen track'ler silinir (bkz. lidar_fusion_node._cleanup_stale_tracks).
MAX_TOTAL_TRACKS = _CFG['MAX_TOTAL_TRACKS']

# Dubalar SABİTTİR — var olan bir track için yeni ölçüm bu kadar "sıçrarsa" (muhtemelen
# hızlı/sert dönüşte fused_yaw'ın anlık hatası, dubanın gerçekten yer değiştirmesi
# DEĞİL) hemen kabul edilmez; ART ARDA JUMP_CONFIRM_COUNT kez AYNI yeni konumda tutarlı
# gelirse gerçek bir yeniden-konumlama sayılır (bkz. apply_track_position).
MAX_TRACK_JUMP_M = _CFG['MAX_TRACK_JUMP_M']
JUMP_CONFIRM_COUNT = _CFG['JUMP_CONFIRM_COUNT']

FUSION_DISTANCE_M = _CFG['FUSION_DISTANCE_M']
# En yakın İKİ track adayı birbirine bu mesafeden daha yakınsa (hangisi olduğu
# belirsizse) kamera oyu HİÇBİRİNE verilmez — küçük bir kamera/derinlik kayması oyun
# yanlış/komşu bir kümeye "savrulmasını" önler (bkz. find_unambiguous_nearest).
FUSION_AMBIGUITY_MARGIN_M = _CFG['FUSION_AMBIGUITY_MARGIN_M']
COLOR_VOTE_THRESHOLD = _CFG['COLOR_VOTE_THRESHOLD']

# --- Kamera/LiDAR node arası poz senkronizasyonu ---
EGO_POSE_STALE_WARN_S = _CFG['EGO_POSE_STALE_WARN_S']

# --- Yönelim (yaw) kaynak seçimi: OAK IMU + Pixhawk füzyon / tek kaynağa düşme ---
IMU_STALE_TIMEOUT_S = _CFG['IMU_STALE_TIMEOUT_S']
ODOM_STALE_TIMEOUT_S = _CFG['ODOM_STALE_TIMEOUT_S']

# --- Koridor (Kenar-Takip) Navigasyonu — Nav2/GPS YOK ---
# Renk şeması: SARI = orta/engel, KIRMIZI+YEŞİL (ikisi birlikte) = kenar/koridor dubası.
LOOK_AHEAD_M = _CFG['LOOK_AHEAD_M']
WAYPOINT_OFFSET_M = _CFG['WAYPOINT_OFFSET_M']
GOAL_TOLERANCE_M = _CFG['GOAL_TOLERANCE_M']
MIN_SAFE_DIST_M = _CFG['MIN_SAFE_DIST_M']
# Sadece TEK taraf (kırmızı YA DA yeşil) önde görülüp diğer taraf hiç görülmediğinde
# (dubalar seyrekse sık karşılaşılan durum) kanalın tahmini yarı-genişliği — görünen
# tek dubadan bu kadar karşı yöne kaydırılarak hedef üretilir (bkz. compute_corridor_waypoint).
ASSUMED_HALF_WIDTH_M = _CFG['ASSUMED_HALF_WIDTH_M']

# --- Sürüş P-Kontrolü ---
STEER_KP = _CFG['STEER_KP']
MAX_STEER_AXIS = _CFG['MAX_STEER_AXIS']
MIN_THROTTLE_AXIS = _CFG['MIN_THROTTLE_AXIS']

# --- Dönüş Hızı Governor'ı (bkz. govern_turn_axis) ---
TARGET_YAW_RATE_DEG_S = _CFG['TARGET_YAW_RATE_DEG_S']
YAW_RATE_GOVERNOR_STEP = _CFG['YAW_RATE_GOVERNOR_STEP']
YAW_RATE_FILTER_ALPHA = _CFG['YAW_RATE_FILTER_ALPHA']

# --- Sürüş Watchdog'u ---
DRIVE_WATCHDOG_LIDAR_TIMEOUT_S = _CFG['DRIVE_WATCHDOG_LIDAR_TIMEOUT_S']

# --- Motor Çıkışı (RC Override) ---
KANAL_DUMEN = _CFG['KANAL_DUMEN']
KANAL_GAZ = _CFG['KANAL_GAZ']
DUMEN_YONU = _CFG['DUMEN_YONU']
RC_PWM_CENTER = _CFG['RC_PWM_CENTER']
RC_PWM_SCALE = _CFG['RC_PWM_SCALE']
RC_PWM_DEADZONE = _CFG['RC_PWM_DEADZONE']
RC_PWM_MIN = _CFG['RC_PWM_MIN']
RC_PWM_MAX = _CFG['RC_PWM_MAX']

# --- Local Costmap ---
COSTMAP_RES = _CFG['COSTMAP_RES']
COSTMAP_SIZE = _CFG['COSTMAP_SIZE']


# ---------------------------------------------------------------------------
# SAF FONKSİYONLAR — LiDAR ön işleme, kümeleme, ego-motion (bağımsız test edilebilir)
# ---------------------------------------------------------------------------

def point_in_range(x_body: float, y_body: float, radius: float = MAP_RADIUS_M) -> bool:
    """Gövde çerçevesindeki bir noktanın gemi merkezli dairesel algılama alanı
    (yarıçap=radius) içinde olup olmadığını test eder."""
    return math.hypot(x_body, y_body) <= radius


def project_lidar_to_2d(xyz_points: np.ndarray,
                         z_min: float = LIDAR_Z_MIN,
                         z_max: float = LIDAR_Z_MAX,
                         radius: float = MAP_RADIUS_M) -> np.ndarray:
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
    """Bir taramadaki nokta sayısı max_points'i aşarsa rastgele altörnekleme yapar.

    `cluster_lidar_points` tipik seyrek Velodyne taramalarında hızlı olsa da, sahil
    kenarı gibi anormal yoğun sahnelerde (yağmur, sıçrama, çok yakın büyük bir yüzey)
    nokta sayısı patlarsa en kötü durum yine O(n²)'ye yaklaşabilir. Bu fonksiyon, işlem
    süresini sabit bir üst sınırda tutmak için son çare bir güvenlik supabıdır — normal
    taramalarda (nokta sayısı sınırın altında) devreye girmez, hiçbir şey değişmez."""
    n = len(points_2d)
    if n <= max_points:
        return points_2d
    rng = rng or np.random.default_rng()
    idx = rng.choice(n, size=max_points, replace=False)
    return points_2d[idx]


def cluster_lidar_points(points_2d: np.ndarray,
                          cluster_distance: float = LIDAR_CLUSTER_DISTANCE,
                          min_points: int = LIDAR_MIN_CLUSTER_POINTS) -> list:
    """Mesafe-eşikli, tohum-merkezli açgözlü kümeleme (orijinal `oak.py` mantığıyla
    BİREBİR aynı davranış): sırayla ilk kullanılmamış noktayı "tohum" alıp ondan
    `cluster_distance` içindeki tüm kullanılmamış noktaları TEK SEFERDE o kümeye
    toplar (transitif/bağlantılı-bileşen DEĞİL — sadece tohuma olan mesafeye bakar).
    Bu davranış kasıtlı korunuyor: gerçek transitif bağlı-bileşen kümeleme, yoğun su
    yüzeyi gürültüsünde birbirinden uzak dubaları tek bir dev kümede birleştirebiliyor
    (test edildi, bkz. commit notu) — oysa açgözlü tohum yaklaşımı bunu doğal olarak
    sınırlıyor.

    Eskiden bu, her tohum için TÜM noktalara `np.linalg.norm` ile mesafe hesaplayan
    saf Python/numpy bir O(n²) döngüydü — Velodyne'in binlerce nokta içeren taramaları
    ile LiDAR callback'ini saniyelerce kilitleyip hem ego-motion/track güncellemelerini
    geciktirebiliyor hem de bazı taramaların hiç işlenememesine (dolayısıyla bazı
    dubaların RViz'de hiç görünmemesine) yol açabiliyordu.

    Yeni uygulama SADECE komşu aramasını `cKDTree.query_ball_point` ile hızlandırır
    (tipik seyrek nokta bulutlarında O(n log n)'e yakın); kümeleme SEMANTİĞİ ve
    girdi/çıktı sözleşmesi (liste of {'center','size'}) eskisiyle birebir aynıdır.
    """
    n = len(points_2d)
    if n == 0:
        return []

    tree = cKDTree(points_2d)
    used = np.zeros(n, dtype=bool)
    clusters = []
    for i in range(n):
        if used[i]:
            continue
        # query_ball_point mesafe<=r döndürür (eskisi gibi KESİN "<" için post-filtre).
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


def find_unambiguous_nearest(candidates: list, query_x: float, query_y: float,
                              max_distance: float,
                              ambiguity_margin: float = FUSION_AMBIGUITY_MARGIN_M):
    """(x, y, ref) üçlülerinden oluşan candidates listesinde (query_x, query_y)'ye en
    yakın olanı bulur — AMA sadece bu eşleşme "net" ise. İki durumda None döner (hiçbir
    şeye eşleşme yapılmaz):
      1) en yakın aday max_distance'tan uzaksa,
      2) en yakın İKİ aday birbirine ambiguity_margin'den daha yakın mesafedeyse (yani
         hangisinin gerçek eşleşme olduğu belirsizse).
    Bu ikinci kural, küçük bir konum kaymasının (kamera/derinlik gürültüsü) oyu yanlış/
    komşu bir kümeye kaydırmasını önler — net değilse hiç oy verilmez, yanlış yere
    verilmez."""
    if not candidates:
        return None
    dists = sorted(
        (math.hypot(cx - query_x, cy - query_y), ref) for (cx, cy, ref) in candidates
    )
    best_d, best_ref = dists[0]
    if best_d > max_distance:
        return None
    if len(dists) > 1:
        second_d, _ = dists[1]
        if second_d <= max_distance and (second_d - best_d) < ambiguity_margin:
            return None
    return best_ref


def apply_track_position(track: dict, wx: float, wy: float,
                          max_jump: float = MAX_TRACK_JUMP_M,
                          confirm_count: int = JUMP_CONFIRM_COUNT,
                          match_radius: float = TRACK_MATCH_RADIUS_M) -> None:
    """Var olan bir track'in (x,y) konumunu günceller — dubalar SABİT olduğu için, yeni
    ölçüm mevcut konumdan MAX_TRACK_JUMP_M'den fazla "sıçrıyorsa" bu muhtemelen dubanın
    yer değiştirmesi DEĞİL, hızlı/sert bir dönüş sırasında fused_yaw'ın anlık hatasıdır —
    hemen kabul edilmez. ART ARDA confirm_count kez AYNI yeni konumda tutarlı gelirse
    (tek seferlik anlık hata değilse) gerçek bir yeniden-konumlama sayılıp kabul edilir.

    track dict'i DOĞRUDAN mutasyona uğratır (gerekli anahtarlar: 'x','y','pending_x',
    'pending_y','pending_count' — bkz. lidar_fusion_node._match_or_create_track)."""
    jump = math.hypot(wx - track['x'], wy - track['y'])
    if jump <= max_jump:
        track['x'], track['y'] = wx, wy
        track['pending_x'] = track['pending_y'] = None
        track['pending_count'] = 0
        return

    if (track['pending_x'] is not None
            and math.hypot(wx - track['pending_x'], wy - track['pending_y']) <= match_radius):
        track['pending_count'] += 1
    else:
        track['pending_x'], track['pending_y'] = wx, wy
        track['pending_count'] = 1

    if track['pending_count'] >= confirm_count:
        track['x'], track['y'] = wx, wy
        track['pending_x'] = track['pending_y'] = None
        track['pending_count'] = 0


def estimate_translation(prev_centers: list, curr_centers: list, dyaw: float,
                          match_radius: float = EGO_MATCH_RADIUS_M) -> tuple:
    """Rotasyon (dyaw) bilindiği için sadece öteleme çözülür: önceki karenin kümeleri
    -dyaw kadar döndürülüp şimdiki kümelerle en-yakın-komşu eşleştirilir; eşleşen
    çiftlerin medyan kayması gövde-çerçevesinde (dx_body, dy_body) döndürülür.
    Dönen 3. değer, öteleme kestiriminin kaç eşleşmeye dayandığıdır (güven göstergesi)."""
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
# GATE-GEÇİŞ NAVİGASYONU (waypoint_generator.py mantığı, Nav2/GPS'siz) — SAF
# ---------------------------------------------------------------------------

def is_in_front_of(px: float, py: float, rx: float, ry: float, ref_yaw: float) -> bool:
    """(px,py) noktası (rx,ry)'den ref_yaw yönüne göre ÖNDE mi? (waypoint_generator.py
    `_in_front_of` ile birebir aynı)."""
    dx, dy = px - rx, py - ry
    return (dx * math.cos(ref_yaw) + dy * math.sin(ref_yaw)) > 0


def find_side_edges(obstacle: tuple, ref_yaw: float, edge_buoys: list,
                     max_dist: float = LOOK_AHEAD_M):
    """obstacle (bir engel YA DA geminin kendi konumu) etrafında, ref_yaw'a göre SAĞ ve
    SOL en yakın kenar (kırmızı+yeşil havuzu) dubalarını bulur — waypoint_generator.py
    `_find_side_greens`'in kırmızı+yeşil birlikte havuzlanmış genel hali. max_dist
    dışındakiler yok sayılır. Döner: (left, right) — her biri (x,y) tuple'ı ya da None."""
    ox, oy = obstacle
    right_x, right_y = math.sin(ref_yaw), -math.cos(ref_yaw)

    left_candidates, right_candidates = [], []
    for (ex, ey) in edge_buoys:
        if math.hypot(ex - ox, ey - oy) > max_dist:
            continue
        dx, dy = ex - ox, ey - oy
        dot = dx * right_x + dy * right_y
        (right_candidates if dot > 0 else left_candidates).append((ex, ey))

    left = (min(left_candidates, key=lambda b: math.hypot(b[0] - ox, b[1] - oy))
            if left_candidates else None)
    right = (min(right_candidates, key=lambda b: math.hypot(b[0] - ox, b[1] - oy))
             if right_candidates else None)
    return left, right


def compute_corridor_waypoint(ship_xy: tuple, ref_yaw: float,
                               edge_buoys: list, yellow_buoys: list,
                               look_ahead: float = LOOK_AHEAD_M,
                               waypoint_offset: float = WAYPOINT_OFFSET_M,
                               assumed_half_width: float = ASSUMED_HALF_WIDTH_M,
                               min_safe_dist: float = MIN_SAFE_DIST_M):
    """SÜREKLİ kenar-koridoru takibi — eski `compute_gate_waypoint`'in yerini alır.

    Eski sürüm SADECE önde bir SARI (engel) varsa devreye giriyordu (yakınında sarı
    olmayan kırmızı/yeşil dubalar tamamen yok sayılıyordu) ve referans yön olarak
    hedef girilirken BİR KEZ hesaplanıp donan `mission_yaw`'ı kullanıyordu. İki sorunu
    vardı: (1) seyrek dubalı bir parkurda sarı çoğu zaman önde bulunamıyor, bu durumda
    gemi kırmızı/yeşil dubaları hiç görmeden doğrudan nihai hedefe gidiyordu; (2) U gibi
    keskin dönüşlü bir parkurda geminin dönmesi gereken açı büyürken sabit mission_yaw
    hiç güncellenmediği için "önde" testi geminin GERÇEK anlık yönelmesiyle tutarsız
    kalıyordu.

    Bu sürüm HER tick'te (sarı olsun olmasın) en yakın ÖNDEKİ kırmızı+yeşil çiftinin
    ortasına yönelir — ref_yaw olarak çağıranın GÜNCEL fused_yaw'ı vermesi beklenir
    (mission_yaw gibi dondurulmuş bir açı DEĞİL), böylece parkur ne kadar keskin
    dönerse dönsün "önde" testi geminin o anki gerçek yönelmesiyle birlikte kayar.
    Sadece TEK taraf görülüyorsa (dubalar seyrekse sık rastlanır) assumed_half_width
    kadar karşı yöne kaydırılmış bir hedef üretilir. Sarı dubalar artık bir TETİKLEYİCİ
    değil, sadece (kırmızı/yeşil ile birlikte) hesaplanan NİHAİ hedefi min_safe_dist'ten
    yakınsa iten bir güvenlik listesidir.

    BİLİNEN SINIR: bu itme SADECE hesaplanan hedef NOKTASINA yakın engelleri kapsar —
    hedef uzaktaysa (örn. önde yakın kenar dubası yoksa) ve bir sarı gemi ile o uzak
    hedef arasındaki DÜZ HAT üzerinde duruyorsa (hedefin kendisine yakın olmasa bile)
    bu sürüm onu atlayabilir. Yol-üzeri engel kontrolü denendi ama P-kontrolle etkileşip
    kararsız bir döngüye yol açtığı simülasyonla doğrulandı (bkz. algorithm_simulation/
    index.html) — bilinçli olarak GERİ ALINDI, ilerideki bir iyileştirme konusu.

    Önde hiç kırmızı/yeşil yoksa None döner (çağıran doğrudan nihai hedefe yönelmeli —
    parkurda uzun bir boşluk anlamına gelir)."""
    rx, ry = ship_xy
    ahead_edges = [
        b for b in edge_buoys
        if is_in_front_of(b[0], b[1], rx, ry, ref_yaw) and math.hypot(b[0] - rx, b[1] - ry) < look_ahead
    ]
    if not ahead_edges:
        return None

    left, right = find_side_edges(ship_xy, ref_yaw, ahead_edges, max_dist=look_ahead)
    if left is None and right is None:
        return None

    right_x, right_y = math.sin(ref_yaw), -math.cos(ref_yaw)
    if left is None:
        mid_x, mid_y = right[0] - right_x * assumed_half_width, right[1] - right_y * assumed_half_width
    elif right is None:
        mid_x, mid_y = left[0] + right_x * assumed_half_width, left[1] + right_y * assumed_half_width
    else:
        mid_x, mid_y = (left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0

    wx = mid_x + math.cos(ref_yaw) * waypoint_offset
    wy = mid_y + math.sin(ref_yaw) * waypoint_offset

    if not is_in_front_of(wx, wy, rx, ry, ref_yaw):
        wx = rx + math.cos(ref_yaw) * waypoint_offset
        wy = ry + math.sin(ref_yaw) * waypoint_offset

    for (bx, by) in (edge_buoys + yellow_buoys):
        d = math.hypot(wx - bx, wy - by)
        if d < min_safe_dist:
            vx, vy = wx - bx, wy - by
            vlen = math.hypot(vx, vy) or 0.001
            wx = bx + (vx / vlen) * min_safe_dist
            wy = by + (vy / vlen) * min_safe_dist

    return (wx, wy)


# ---------------------------------------------------------------------------
# MOTOR ÇIKIŞI (RC Override mixing/PWM) — SAF
# ---------------------------------------------------------------------------

def calculate_pwm(axis_val: float, center: int = RC_PWM_CENTER, scale: int = RC_PWM_SCALE,
                   deadzone: int = RC_PWM_DEADZONE, pwm_min: int = RC_PWM_MIN,
                   pwm_max: int = RC_PWM_MAX) -> int:
    """RC-override script'indeki `calculate_pwm` ile birebir aynı: [-1,1] civarı bir
    eksen değerini PWM'e çevirir, merkeze yakınsa tam nötr döner, sonucu
    [pwm_min, pwm_max] aralığına kırpar."""
    val = center + int(axis_val * scale)
    if abs(val - center) < deadzone:
        return center
    return max(pwm_min, min(pwm_max, val))


def mix_gaz_dumen(sol: float, sag: float, dumen_yonu: int = DUMEN_YONU) -> tuple:
    """RC-override script'indeki mixing formülünün aynısı: istenen sol/sağ motor
    değerlerinden (ArduPilot'un kendi skid-steer mixer'inin CH1/CH3'ten beklediği)
    gaz/dümen eksenlerini üretir. Döner: (gaz, dumen)."""
    gaz = (sol + sag) / 2.0
    dumen = ((sol - sag) / 2.0) * dumen_yonu
    return gaz, dumen


# ---------------------------------------------------------------------------
# DÖNÜŞ HIZI GOVERNOR'I (kapalı çevrim yaw-rate kontrolü) — SAF
# ---------------------------------------------------------------------------

def low_pass_filter(prev_value: float, new_value: float, alpha: float) -> float:
    """Standart alçak-geçiren filtre: new_value'ya alpha kadar, prev_value'ya (1-alpha)
    kadar ağırlık verir. KÜÇÜK alpha = DAHA FAZLA yumuşatma/gecikme. Burada ham gyro
    (yaw rate) ölçümünü, suyun ani tepki vermeyen dinamiğine uygun şekilde yumuşatmak
    için kullanılır (bkz. govern_turn_axis) — tek bir gürültülü örneğin governor'ı
    yanlış yönlendirmesini önler."""
    return alpha * new_value + (1.0 - alpha) * prev_value


def govern_turn_axis(desired_turn: float, filtered_yaw_rate_deg_s: float,
                      target_yaw_rate_deg_s: float = TARGET_YAW_RATE_DEG_S,
                      ramp_step: float = YAW_RATE_GOVERNOR_STEP,
                      max_axis: float = MAX_STEER_AXIS,
                      prev_governed_mag: float = 0.0) -> tuple:
    """Heading P-kontrolünün istediği dümen ekseni (desired_turn) AÇIK ÇEVRİMDİR — sadece
    hedefe olan açı hatasına bakar, geminin GERÇEKTE ne kadar hızlı döndüğüne değil. Akıntı/su
    direnci yüzünden fiili dönüş hızı komut edilenden yavaş kalabilir. Bu fonksiyon KAPALI
    ÇEVRİM bir governor katmanıdır: ölçülen (ÖNCEDEN low_pass_filter'dan geçirilmiş — suyun
    gecikmeli dinamiğine uygun) yaw rate hedefin (target_yaw_rate_deg_s) altındaysa, dümen
    ekseninin büyüklüğü desired_turn'ü AŞMAYACAK şekilde kademeli (ramp_step/tick) artırılır;
    hedefe ulaşıldıysa/aşıldıysa kademeli geri çekilir. Yani asla P-kontrolün istediğinden
    FAZLA dönüş komutu üretmez — sadece gerçek dönüş hızının hedefe ULAŞMASINI hızlandırır.

    prev_governed_mag: bir önceki tick'te dönen governed büyüklük (çağıran bunu saklamalı,
    bir sonraki çağrıya prev_governed_mag olarak geri vermelidir — bkz.
    lidar_fusion_node._gate_drive_hesapla). Döner: (governed_turn, yeni_governed_mag)."""
    desired_mag = min(abs(desired_turn), max_axis)
    if desired_mag < 1e-6:
        return 0.0, 0.0

    sign = 1.0 if desired_turn > 0 else -1.0
    if filtered_yaw_rate_deg_s < target_yaw_rate_deg_s:
        governed_mag = min(prev_governed_mag + ramp_step, desired_mag, max_axis)
    else:
        governed_mag = max(prev_governed_mag - ramp_step, 0.0)
        governed_mag = min(governed_mag, desired_mag)

    return sign * governed_mag, governed_mag
