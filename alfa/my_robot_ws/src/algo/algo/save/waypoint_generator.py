"""
waypoint_generator.py
---------------------
Algoritma:
1. Başlangıç konumu /odom'dan otomatik alınır
2. Kullanıcı sadece bitiş (B) girer
3. A→B doğrultusuna göre önündeki sarı (engel) duba bulunur
4. O dubanın sağ/sol yeşil (kenar) dubaları bulunur (mesafe filtreli)
5. Geniş tarafa ara hedef koyulur, engelin ötesine
6. Nav2'ye gönderilir, ulaşılınca tekrar 3'e dönülür
7. B'ye ulaşana kadar devam eder

Duba kaynağı: withmap.py → /buoys/yellow + /buoys/green (dinamik, canlı)
map_parser.py artık kullanılmıyor.
"""

import math
import threading
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# ── Ayarlar ──────────────────────────────────────────────────────────────────
LOOK_AHEAD_M      = 20.0
WAYPOINT_OFFSET_M =  2.0
GOAL_TOLERANCE_M  =  1.5
GOAL_TIMEOUT_S    = 60.0
MIN_SAFE_DIST_M   =  2.5
SIDE_GREEN_MAX_M  =  8.0
# ─────────────────────────────────────────────────────────────────────────────


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def pointcloud2_to_list(msg):
    """PointCloud2 mesajini (x, y) listesine donustur"""
    pts = []
    for p in pc2.read_points(msg, field_names=('x', 'y'), skip_nans=True):
        pts.append((float(p[0]), float(p[1])))
    return pts


class WaypointGenerator(Node):

    def __init__(self):
        super().__init__('waypoint_generator')

        # Duba listeleri — withmap.py'den canli gelecek
        self.green_buoys  = []
        self.yellow_buoys = []
        self._buoy_lock   = threading.Lock()
        self._buoys_ready = threading.Event()

        # Canli duba topic'leri
        self.create_subscription(PointCloud2, '/buoys/yellow', self._yellow_cb, 10)
        self.create_subscription(PointCloud2, '/buoys/green',  self._green_cb,  10)
        self.get_logger().info('Duba verileri bekleniyor: /buoys/yellow + /buoys/green')

        # Konum
        self.pose = None
        self._pose_lock = threading.Lock()
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal = None

        self.get_logger().info('Konum bekleniyor (/odom)...')
        self._mission_thread = threading.Thread(target=self._mission_thread_fn, daemon=True)
        self._mission_thread.start()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _yellow_cb(self, msg):
        pts = pointcloud2_to_list(msg)
        with self._buoy_lock:
            self.yellow_buoys = pts
        if pts:
            self._buoys_ready.set()

    def _green_cb(self, msg):
        pts = pointcloud2_to_list(msg)
        with self._buoy_lock:
            self.green_buoys = pts
        if pts:
            self._buoys_ready.set()

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        with self._pose_lock:
            self.pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw
            )

    # ── Mission thread ────────────────────────────────────────────────────────

    def _mission_thread_fn(self):
        import time

        while rclpy.ok():
            with self._pose_lock:
                pose = self.pose
            if pose is not None:
                break
            time.sleep(0.1)

        ax, ay, ayaw = pose
        self.get_logger().info(
            f'Baslangic: x={ax:.2f} y={ay:.2f} yaw={math.degrees(ayaw):.1f} deg'
        )

        # withmap'ten ilk duba verisi gelene kadar bekle (max 30s)
        self.get_logger().info('withmap duba verisi bekleniyor...')
        self._buoys_ready.wait(timeout=30.0)

        with self._buoy_lock:
            ny = len(self.yellow_buoys)
            ng = len(self.green_buoys)
        self.get_logger().info(f'Duba verisi alindi — Sari: {ny}, Yesil: {ng}')

        self.goal = self._get_goal_input()
        self.get_logger().info(f'Hedef: {self.goal}')

        self.mission_yaw = math.atan2(
            self.goal[1] - ay,
            self.goal[0] - ax
        )
        self.get_logger().info(
            f'Gorev yonu: {math.degrees(self.mission_yaw):.1f} deg'
        )

        self._run_mission()

    # ── Kullanıcı girişi ──────────────────────────────────────────────────────

    def _get_goal_input(self):
        print('\n=== WAYPOINT GENERATOR ===')
        print('Koordinat formati: x y')
        print('Ornek: 80.0 20.0\n')
        while True:
            try:
                raw = input('Bitis (B) [x y]: ').strip().split()
                return (float(raw[0]), float(raw[1]))
            except Exception:
                print('Hatali giris, tekrar dene.')

    # ── Ana görev döngüsü ──────────────────────────────────────────────────────

    def _run_mission(self):
        import time
        while rclpy.ok():
            with self._pose_lock:
                pose = self.pose
            if pose is None:
                time.sleep(0.1)
                continue

            if dist(pose[:2], self.goal) < GOAL_TOLERANCE_M:
                self.get_logger().info('HEDEFE ULASILDI!')
                os._exit(0)

            remaining = dist(pose[:2], self.goal)
            if remaining < 8.2:
                self.get_logger().info(
                    f'Hedefe {remaining:.1f}m kaldi, direkt gidiliyor.'
                )
                self._send_goal_and_wait(self.goal[0], self.goal[1], self.mission_yaw)
                self.get_logger().info('HEDEFE ULASILDI!')
                os._exit(0)

            waypoint = self._compute_waypoint(pose)

            if waypoint is None:
                self.get_logger().info('Onde engel yok, direkt hedefe.')
                self._send_goal_and_wait(self.goal[0], self.goal[1], self.mission_yaw)
            else:
                wx, wy, wyaw = waypoint
                self.get_logger().info(
                    f'Ara hedef x:{wx:.2f} y:{wy:.2f} yaw:{math.degrees(wyaw):.1f}'
                )
                self._send_goal_and_wait(wx, wy, wyaw)

    # ── Waypoint hesaplama ────────────────────────────────────────────────────

    def _compute_waypoint(self, pose):
        rx, ry, _ = pose
        ref_yaw = self.mission_yaw

        with self._buoy_lock:
            yellow_buoys = list(self.yellow_buoys)
            green_buoys  = list(self.green_buoys)

        nearby_yellow = [
            b for b in yellow_buoys
            if self._in_front_of(b, rx, ry, ref_yaw)
            and dist((rx, ry), b) < LOOK_AHEAD_M
        ]

        if not nearby_yellow:
            return None

        nearest_yellow = min(nearby_yellow, key=lambda b: dist((rx, ry), b))
        yx, yy = nearest_yellow
        self.get_logger().info(
            f'Engel duba: ({yx:.1f}, {yy:.1f}), '
            f'mesafe: {dist((rx,ry), nearest_yellow):.1f}m'
        )

        left_green, right_green = self._find_side_greens(
            nearest_yellow, ref_yaw, green_buoys
        )

        if left_green is None and right_green is None:
            self.get_logger().warn('Kenar duba bulunamadi, engeli duz geciyorum.')
            wx = yx + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
            wy = yy + math.sin(ref_yaw) * WAYPOINT_OFFSET_M
            return (wx, wy, ref_yaw)

        if left_green is None:
            chosen, side = right_green, 'SAG'
        elif right_green is None:
            chosen, side = left_green, 'SOL'
        else:
            d_left  = dist(nearest_yellow, left_green)
            d_right = dist(nearest_yellow, right_green)
            self.get_logger().info(
                f'Sol kenar: {d_left:.1f}m | Sag kenar: {d_right:.1f}m'
            )
            if d_left >= d_right:
                chosen, side = left_green, 'SOL (genis)'
            else:
                chosen, side = right_green, 'SAG (genis)'

        self.get_logger().info(f'Gecis yonu: {side}')

        mid_x = (yx + chosen[0]) / 2.0
        mid_y = (yy + chosen[1]) / 2.0

        wx = mid_x + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
        wy = mid_y + math.sin(ref_yaw) * WAYPOINT_OFFSET_M

        if not self._in_front_of((wx, wy), rx, ry, ref_yaw):
            wx = rx + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
            wy = ry + math.sin(ref_yaw) * WAYPOINT_OFFSET_M

        all_buoys = green_buoys + yellow_buoys
        for bx, by in all_buoys:
            if dist((wx, wy), (bx, by)) < MIN_SAFE_DIST_M:
                self.get_logger().warn(
                    f'Ara hedef dubaya cok yakin '
                    f'({dist((wx,wy),(bx,by)):.1f}m), duzeltiliyor.'
                )
                vx = wx - bx
                vy = wy - by
                vlen = math.hypot(vx, vy) or 0.001
                wx = bx + (vx / vlen) * MIN_SAFE_DIST_M
                wy = by + (vy / vlen) * MIN_SAFE_DIST_M

        wyaw = math.atan2(self.goal[1] - wy, self.goal[0] - wx)
        return (wx, wy, wyaw)

    def _in_front_of(self, buoy, rx, ry, ref_yaw):
        dx = buoy[0] - rx
        dy = buoy[1] - ry
        return (dx * math.cos(ref_yaw) + dy * math.sin(ref_yaw)) > 0

    def _find_side_greens(self, yellow_buoy, ref_yaw, green_buoys):
        yx, yy = yellow_buoy
        right_x =  math.sin(ref_yaw)
        right_y = -math.cos(ref_yaw)

        left_greens  = []
        right_greens = []

        for gx, gy in green_buoys:
            if dist((gx, gy), yellow_buoy) > SIDE_GREEN_MAX_M:
                continue
            dx = gx - yx
            dy = gy - yy
            dot = dx * right_x + dy * right_y
            if dot > 0:
                right_greens.append((gx, gy))
            else:
                left_greens.append((gx, gy))

        left_green  = (
            min(left_greens,  key=lambda b: dist(yellow_buoy, b))
            if left_greens else None
        )
        right_green = (
            min(right_greens, key=lambda b: dist(yellow_buoy, b))
            if right_greens else None
        )
        return left_green, right_green

    # ── Nav2 ──────────────────────────────────────────────────────────────────

    def _send_goal_and_wait(self, x, y, yaw):
        import time
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server bulunamadi!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose_stamped(x, y, yaw)

        send_future = self._nav_client.send_goal_async(goal_msg)

        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Hedef reddedildi!')
            return

        last_log = 0.0
        start_time = time.time()

        while rclpy.ok():
            time.sleep(0.1)
            with self._pose_lock:
                pose = self.pose
            if pose is None:
                continue

            d = dist(pose[:2], (x, y))
            now = time.time()
            elapsed = now - start_time

            if now - last_log > 2.0:
                self.get_logger().info(
                    f'Hedefe mesafe: {d:.1f}m | Gecen: {elapsed:.0f}s'
                )
                last_log = now

            if d < GOAL_TOLERANCE_M:
                self.get_logger().info(
                    f'Tolerans icinde ({d:.1f}m), sonraki adima geciliyor.'
                )
                goal_handle.cancel_goal_async()
                time.sleep(0.5)
                break

            if elapsed > GOAL_TIMEOUT_S:
                self.get_logger().warn(
                    f'TIMEOUT! Kalan: {d:.1f}m, skip ediliyor.'
                )
                goal_handle.cancel_goal_async()
                time.sleep(0.5)
                break

    def _make_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    
    # Spin'i ayrı thread'de çalıştır, mission thread ile paralel dönsün
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
