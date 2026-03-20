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
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from algo.map_parser import MapParser

# ── Ayarlar ──────────────────────────────────────────────────────────────────
MAP_IMAGE         = '/home/gauss/Desktop/my_robot_ws/src/boat_navigation/maps/costmap_colored.png'
RESOLUTION        = 0.05
ORIGIN            = [-64.2, -25.5, 0.0]

LOOK_AHEAD_M      = 20.0  # bakış penceresi (metre) — artırıldı
WAYPOINT_OFFSET_M =  2.0  # engelin ötesine konacak ara hedef mesafesi
GOAL_TOLERANCE_M  =  1.5  # hedefe bu kadar yaklaşınca "ulaştı" say
GOAL_TIMEOUT_S    = 60.0  # bu kadar saniye geçerse hedefi skip et
MIN_SAFE_DIST_M   =  2.5  # ara hedef dubaya bu kadardan yakın olamaz
SIDE_GREEN_MAX_M  =  8.0  # sarı dubadan bu kadar uzaktaki yeşilleri "yan kenar" say
# ─────────────────────────────────────────────────────────────────────────────


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointGenerator(Node):

    def __init__(self):
        super().__init__('waypoint_generator')

        # Harita parse
        self.get_logger().info('Harita okunuyor...')
        parser = MapParser(MAP_IMAGE, RESOLUTION, ORIGIN)
        self.green_buoys  = parser.get_green_buoys()
        self.yellow_buoys = parser.get_yellow_buoys()
        self.get_logger().info(
            f'Yeşil duba: {len(self.green_buoys)}, Sarı duba: {len(self.yellow_buoys)}'
        )

        # Konum
        self.pose = None  # (x, y, yaw)
        self._pose_lock = threading.Lock()
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Hedef — thread'den set edilecek
        self.goal = None

        # input() ayrı thread'de çalışsın, spin bloklanmasın
        self.get_logger().info('Konum bekleniyor (/odom)...')
        self._mission_thread = threading.Thread(target=self._mission_thread_fn, daemon=True)
        self._mission_thread.start()

    # ── Callbacks ─────────────────────────────────────────────────────────────

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
        # Odom gelene kadar bekle
        import time
        while rclpy.ok():
            with self._pose_lock:
                pose = self.pose
            if pose is not None:
                break
            time.sleep(0.1)

        ax, ay, ayaw = pose
        self.get_logger().info(f'Başlangıç: x={ax:.2f} y={ay:.2f} yaw={math.degrees(ayaw):.1f}°')

        # Kullanıcıdan hedef al — burada input() güvenli, spin ayrı thread'de
        self.goal = self._get_goal_input()
        self.get_logger().info(f'Hedef: {self.goal}')

        # A→B görev yönü — tüm görev boyunca sabit referans
        self.mission_yaw = math.atan2(
            self.goal[1] - ay,
            self.goal[0] - ax
        )
        self.get_logger().info(f'Görev yönü: {math.degrees(self.mission_yaw):.1f}°')

        self._run_mission()

    # ── Kullanıcı girişi ──────────────────────────────────────────────────────

    def _get_goal_input(self):
        print('\n=== WAYPOINT GENERATOR ===')
        print('Koordinat formatı: x y')
        print('Örnek: 80.0 20.0\n')
        while True:
            try:
                raw = input('Bitiş (B) [x y]: ').strip().split()
                return (float(raw[0]), float(raw[1]))
            except Exception:
                print('Hatalı giriş, tekrar dene.')

    # ── Ana görev döngüsü ──────────────────────────────────────────────────────

    def _run_mission(self):
        import time
        while rclpy.ok():
            with self._pose_lock:
                pose = self.pose
            if pose is None:
                time.sleep(0.1)
                continue

            # B'ye ulaştık mı?
            if dist(pose[:2], self.goal) < GOAL_TOLERANCE_M:
                self.get_logger().info('✓ HEDEFE ULAŞILDI!')
                return

            waypoint = self._compute_waypoint(pose)

            if waypoint is None:
                # Önde engel yok, direkt B'ye git
                self.get_logger().info('Önde engel yok → direkt hedefe.')
                self._send_goal_and_wait(self.goal[0], self.goal[1], self.mission_yaw)
            else:
                wx, wy, wyaw = waypoint
                self.get_logger().info(
                    f'Ara hedef → x:{wx:.2f} y:{wy:.2f} yaw:{math.degrees(wyaw):.1f}°'
                )
                self._send_goal_and_wait(wx, wy, wyaw)

    # ── Waypoint hesaplama ────────────────────────────────────────────────────

    def _compute_waypoint(self, pose):
        rx, ry, _ = pose
        ref_yaw = self.mission_yaw

        # Önündeki sarı dubalar (LOOK_AHEAD_M içinde, A→B yönünde)
        nearby_yellow = [
            b for b in self.yellow_buoys
            if self._in_front_of(b, rx, ry, ref_yaw)
            and dist((rx, ry), b) < LOOK_AHEAD_M
        ]

        if not nearby_yellow:
            return None

        # En yakın sarı duba
        nearest_yellow = min(nearby_yellow, key=lambda b: dist((rx, ry), b))
        yx, yy = nearest_yellow
        self.get_logger().info(
            f'Engel duba: ({yx:.1f}, {yy:.1f}), mesafe: {dist((rx,ry), nearest_yellow):.1f}m'
        )

        # Sağ ve sol yeşil kenarları bul (mesafe filtreli)
        left_green, right_green = self._find_side_greens(nearest_yellow, ref_yaw)

        if left_green is None and right_green is None:
            self.get_logger().warn('Kenar duba bulunamadı, engeli düz geçiyorum.')
            wx = yx + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
            wy = yy + math.sin(ref_yaw) * WAYPOINT_OFFSET_M
            return (wx, wy, ref_yaw)

        # Hangi tarafa kayacağız?
        if left_green is None:
            chosen = right_green
            side = 'SAĞ'
        elif right_green is None:
            chosen = left_green
            side = 'SOL'
        else:
            d_left  = dist(nearest_yellow, left_green)
            d_right = dist(nearest_yellow, right_green)
            self.get_logger().info(
                f'Sol kenar: {d_left:.1f}m | Sağ kenar: {d_right:.1f}m'
            )
            if d_left >= d_right:
                chosen = left_green
                side = 'SOL (geniş)'
            else:
                chosen = right_green
                side = 'SAĞ (geniş)'

        self.get_logger().info(f'Geçiş yönü: {side}')

        # Sarı duba ile seçilen yeşil kenarın ortası
        mid_x = (yx + chosen[0]) / 2.0
        mid_y = (yy + chosen[1]) / 2.0

        # Orta noktanın ötesine WAYPOINT_OFFSET_M kadar git
        wx = mid_x + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
        wy = mid_y + math.sin(ref_yaw) * WAYPOINT_OFFSET_M

        # Geri düşüyorsa düzelt
        if not self._in_front_of((wx, wy), rx, ry, ref_yaw):
            wx = rx + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
            wy = ry + math.sin(ref_yaw) * WAYPOINT_OFFSET_M

        # Dubaya çok yakınsa güvenli mesafeye çek
        all_buoys = self.green_buoys + self.yellow_buoys
        for bx, by in all_buoys:
            if dist((wx, wy), (bx, by)) < MIN_SAFE_DIST_M:
                self.get_logger().warn(
                    f'Ara hedef dubaya çok yakın ({dist((wx,wy),(bx,by)):.1f}m), düzeltiliyor.'
                )
                vx = wx - bx
                vy = wy - by
                vlen = math.hypot(vx, vy) or 0.001
                wx = bx + (vx / vlen) * MIN_SAFE_DIST_M
                wy = by + (vy / vlen) * MIN_SAFE_DIST_M

        wyaw = math.atan2(self.goal[1] - wy, self.goal[0] - wx)
        return (wx, wy, wyaw)

    def _in_front_of(self, buoy, rx, ry, ref_yaw):
        """Duba, ref_yaw yönünde önde mi?"""
        dx = buoy[0] - rx
        dy = buoy[1] - ry
        return (dx * math.cos(ref_yaw) + dy * math.sin(ref_yaw)) > 0

    def _find_side_greens(self, yellow_buoy, ref_yaw):
        """
        Sarı dubanın yakınındaki (SIDE_GREEN_MAX_M) sağ/sol yeşil dubaları bul.
        Sağ/sol A→B yönüne göre belirlenir.
        """
        yx, yy = yellow_buoy
        right_x =  math.sin(ref_yaw)
        right_y = -math.cos(ref_yaw)

        left_greens  = []
        right_greens = []

        for gx, gy in self.green_buoys:
            # DÜZELTME: sadece sarı dubaya yakın yeşilleri al
            if dist((gx, gy), yellow_buoy) > SIDE_GREEN_MAX_M:
                continue

            dx = gx - yx
            dy = gy - yy
            dot = dx * right_x + dy * right_y
            if dot > 0:
                right_greens.append((gx, gy))
            else:
                left_greens.append((gx, gy))

        left_green  = min(left_greens,  key=lambda b: dist(yellow_buoy, b)) if left_greens  else None
        right_green = min(right_greens, key=lambda b: dist(yellow_buoy, b)) if right_greens else None

        return left_green, right_green

    # ── Nav2 ──────────────────────────────────────────────────────────────────

    def _send_goal_and_wait(self, x, y, yaw):
        import time
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server bulunamadı!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose_stamped(x, y, yaw)

        send_future = self._nav_client.send_goal_async(goal_msg)

        # Future tamamlanana kadar bekle (spin ayrı thread'de)
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
                self.get_logger().info(f'Hedefe mesafe: {d:.1f}m | Geçen: {elapsed:.0f}s')
                last_log = now

            if d < GOAL_TOLERANCE_M:
                self.get_logger().info(f'✓ Tolerans içinde ({d:.1f}m), sonraki adıma geçiliyor.')
                goal_handle.cancel_goal_async()
                time.sleep(0.5)
                break

            if elapsed > GOAL_TIMEOUT_S:
                self.get_logger().warn(f'⚠ TIMEOUT! Kalan: {d:.1f}m, skip ediliyor.')
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
