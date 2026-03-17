"""
waypoint_generator.py
---------------------
Algoritma:
1. Başlangıç konumu /odom'dan otomatik alınır
2. Kullanıcı sadece bitiş (B) girer
3. A→B doğrultusuna göre önündeki sarı (engel) duba bulunur
4. O dubanın sağ/sol yeşil (kenar) dubaları bulunur
5. Geniş tarafa ara hedef koyulur, engelin ötesine
6. Nav2'ye gönderilir, ulaşılınca tekrar 3'e dönülür
7. B'ye ulaşana kadar devam eder
"""

import math
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

LOOK_AHEAD_M      = 10.0  # bakış penceresi (metre)
WAYPOINT_OFFSET_M =  1.5  # engelin ötesine konacak ara hedef mesafesi
GOAL_TOLERANCE_M  =  1.5  # hedefe bu kadar yaklaşınca "ulaştı" say
GOAL_TIMEOUT_S    = 30.0  # bu kadar saniye geçerse hedefi skip et
MIN_SAFE_DIST_M   =  2.5  # ara hedef dubaya bu kadardan yakın olamaz
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
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Odom gelene kadar bekle, sonra kullanıcıdan B al
        self.get_logger().info('Konum bekleniyor (/odom)...')
        self._init_timer = self.create_timer(0.5, self._wait_for_odom)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        )

    def _wait_for_odom(self):
        if self.pose is None:
            return
        self._init_timer.cancel()

        ax, ay, ayaw = self.pose
        self.get_logger().info(f'Başlangıç konumu alındı: x={ax:.2f} y={ay:.2f} yaw={ayaw:.2f}')

        # Kullanıcıdan bitiş al
        self.goal = self._get_goal_input()
        self.get_logger().info(f'Hedef: {self.goal}')

        # A→B yönünü kaydet (tüm görev boyunca bu yön referans)
        self.mission_yaw = math.atan2(
            self.goal[1] - ay,
            self.goal[0] - ax
        )
        self.get_logger().info(f'Görev yönü (A→B yaw): {math.degrees(self.mission_yaw):.1f}°')

        self._run_mission()

    # ── Kullanıcı girişi ──────────────────────────────────────────────────────

    def _get_goal_input(self):
        print('\n=== WAYPOINT GENERATOR ===')
        print('Koordinat formatı: x y  (yaw otomatik hesaplanır)')
        print('Örnek: 80.0 20.0\n')
        while True:
            try:
                raw = input('Bitiş (B) [x y]: ').strip().split()
                return (float(raw[0]), float(raw[1]))
            except Exception:
                print('Hatalı giriş, tekrar dene.')

    # ── Ana görev döngüsü ──────────────────────────────────────────────────────

    def _run_mission(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.pose is None:
                continue

            # B'ye ulaştık mı?
            if dist(self.pose[:2], self.goal) < GOAL_TOLERANCE_M:
                self.get_logger().info('HEDEFE ULAŞILDI!')
                break

            waypoint = self._compute_waypoint()

            if waypoint is None:
                # Önde engel kalmadı, direkt B'ye git
                self.get_logger().info('Önde engel yok, direkt hedefe gidiliyor.')
                self._send_goal_and_wait(self.goal[0], self.goal[1], self.mission_yaw)
                break
            else:
                wx, wy, wyaw = waypoint
                self.get_logger().info(
                    f'Ara hedef → x:{wx:.2f} y:{wy:.2f} yaw:{math.degrees(wyaw):.1f}°'
                )
                self._send_goal_and_wait(wx, wy, wyaw)

    # ── Waypoint hesaplama ────────────────────────────────────────────────────

    def _compute_waypoint(self):
        rx, ry, _ = self.pose
        ref_yaw = self.mission_yaw  # A→B yönü sabit referans

        # A→B yönünde önündeki sarı dubalar (LOOK_AHEAD_M içinde)
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
        self.get_logger().info(f'Engel duba: ({yx:.1f}, {yy:.1f}), mesafe: {dist((rx,ry), nearest_yellow):.1f}m')

        # Sağ ve sol yeşil kenarları bul
        left_green, right_green = self._find_side_greens(nearest_yellow, ref_yaw)

        if left_green is None and right_green is None:
            self.get_logger().warn('Kenar duba bulunamadı, engeli düz geçiyorum.')
            wx = yx + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
            wy = yy + math.sin(ref_yaw) * WAYPOINT_OFFSET_M
            return (wx, wy, ref_yaw)

        if left_green is None:
            # Sadece sağ var, sola git
            mid_x = (yx + right_green[0]) / 2.0
            mid_y = (yy + right_green[1]) / 2.0
        elif right_green is None:
            # Sadece sol var, sağa git
            mid_x = (yx + left_green[0]) / 2.0
            mid_y = (yy + left_green[1]) / 2.0
        else:
            d_left  = dist(nearest_yellow, left_green)
            d_right = dist(nearest_yellow, right_green)
            self.get_logger().info(
                f'Sol kenar mesafe: {d_left:.1f}m | Sağ kenar mesafe: {d_right:.1f}m'
            )
            if d_left >= d_right:
                # Sol daha geniş → sola kay
                mid_x = (yx + left_green[0]) / 2.0
                mid_y = (yy + left_green[1]) / 2.0
            else:
                # Sağ daha geniş → sağa kay
                mid_x = (yx + right_green[0]) / 2.0
                mid_y = (yy + right_green[1]) / 2.0

        # Engel dubasının yan tarafına (mid) git, oradan ileri yöne WAYPOINT_OFFSET_M kadar
        wx = mid_x + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
        wy = mid_y + math.sin(ref_yaw) * WAYPOINT_OFFSET_M

        # Eğer ara hedef mevcut konumdan geriye düşüyorsa düzelt
        if not self._in_front_of((wx, wy), rx, ry, ref_yaw):
            wx = rx + math.cos(ref_yaw) * WAYPOINT_OFFSET_M
            wy = ry + math.sin(ref_yaw) * WAYPOINT_OFFSET_M

        # Ara hedef herhangi bir dubaya çok yakınsa güvenli mesafeye çek
        all_buoys = self.green_buoys + self.yellow_buoys
        for bx, by in all_buoys:
            if dist((wx, wy), (bx, by)) < MIN_SAFE_DIST_M:
                self.get_logger().warn(
                    f'Ara hedef dubaya çok yakın ({dist((wx,wy),(bx,by)):.1f}m), düzeltiliyor.'
                )
                # Dubadan uzaklaştır: duba→hedef vektörünü uzat
                vx = wx - bx
                vy = wy - by
                vlen = math.hypot(vx, vy) or 0.001
                wx = bx + (vx / vlen) * MIN_SAFE_DIST_M
                wy = by + (vy / vlen) * MIN_SAFE_DIST_M

        # Ara hedefe bakış açısı: B'ye doğru
        wyaw = math.atan2(self.goal[1] - wy, self.goal[0] - wx)

        return (wx, wy, wyaw)

    def _in_front_of(self, buoy, rx, ry, ref_yaw):
        """Duba, ref_yaw yönünde önde mi?"""
        dx = buoy[0] - rx
        dy = buoy[1] - ry
        return (dx * math.cos(ref_yaw) + dy * math.sin(ref_yaw)) > 0

    def _find_side_greens(self, yellow_buoy, ref_yaw):
        """
        Sarı dubanın sağındaki ve solundaki en yakın yeşil dubaları bul.
        Sağ/sol A→B yönüne göre belirlenir.
        """
        yx, yy = yellow_buoy
        # A→B yönüne dik sağ vektör
        right_x =  math.sin(ref_yaw)
        right_y = -math.cos(ref_yaw)

        left_greens  = []
        right_greens = []

        for gx, gy in self.green_buoys:
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
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server bulunamadı!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose_stamped(x, y, yaw)

        send_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Hedef reddedildi!')
            return

        # Nav2'nin kendi result'unu BEKLEME
        # Sadece mesafeyi izle, toleransa girince veya timeout dolunca cancel et
        last_log = 0.0
        start_time = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is None:
                continue

            d = dist(self.pose[:2], (x, y))
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - start_time

            if now - last_log > 2.0:
                self.get_logger().info(f'Hedefe mesafe: {d:.1f}m | Geçen süre: {elapsed:.0f}s')
                last_log = now

            if d < GOAL_TOLERANCE_M:
                self.get_logger().info(
                    f'Tolerans içinde ({d:.1f}m), hedef iptal ediliyor → sonraki adım.'
                )
                goal_handle.cancel_goal_async()
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.1)
                break

            if elapsed > GOAL_TIMEOUT_S:
                self.get_logger().warn(
                    f'TIMEOUT ({GOAL_TIMEOUT_S}s)! Hedef skip ediliyor. Kalan mesafe: {d:.1f}m'
                )
                goal_handle.cancel_goal_async()
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.1)
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
