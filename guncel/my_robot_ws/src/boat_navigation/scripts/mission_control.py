import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np

class GapFollowerMissionControl(Node):
    def __init__(self):
        super().__init__('gap_follower_mission_control')

        # Görev Sınırları (Yarı Otonom Bölge)
        self.start_zone = {'x': -10.0, 'y': 0.0, 'radius': 3.0}
        self.end_zone = {'x': 40.0, 'y': 20.0, 'radius': 5.0}
        self.mission_active = False

        # TF2 Tanımlamaları (Konum almak için)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Lidar Aboneliği
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None

        # Kontrol Döngüsü (1 Hz - Saniyede 1 kez hedef güncellemesi)
        self.timer = self.create_timer(1.0, self.control_loop)

        self.latest_scan = None
        self.current_pose = None

    def scan_callback(self, msg):
        self.latest_scan = msg

    def get_current_pose(self):
        try:
            # map'ten base_link'e olan dönüşümü oku
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Quaternion'dan Yaw açısına çevirme
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (x, y, yaw)
        except TransformException as ex:
            return None

    def find_best_gap_angle(self):
        if not self.latest_scan:
            return 0.0

        ranges = np.array(self.latest_scan.ranges)
        # Sonsuz veya hatalı verileri filtrele
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), self.latest_scan.range_max, ranges)

        # Basit Gap Algoritması: Ön 180 dereceyi tara, engellerin uzak olduğu en geniş bölgeyi bul
        # Gerçek uygulamada burada yeşil ve kırmızı dubaların arasını ölçecek YOLO/PCL mantığın çalışacak
        
        # Şimdilik örnek: Sadece tam ileriye (0 derece) gitmesini varsayalım
        best_angle_relative = 0.0 
        return best_angle_relative

    def control_loop(self):
        pose = self.get_current_pose()
        if not pose:
            return
        
        curr_x, curr_y, curr_yaw = pose

        # Bitiş noktasına gelindi mi?
        dist_to_end = math.hypot(self.end_zone['x'] - curr_x, self.end_zone['y'] - curr_y)
        if dist_to_end < self.end_zone['radius']:
            self.get_logger().info('Bitiş noktasına ulaşıldı. Otonomi durduruluyor.')
            self.mission_active = False
            return

        # Başlangıç noktasına girildiğinde görevi tetikle
        dist_to_start = math.hypot(self.start_zone['x'] - curr_x, self.start_zone['y'] - curr_y)
        if dist_to_start < self.start_zone['radius']:
            self.mission_active = True

        if self.mission_active:
            # 1. En iyi boşluğun açısını bul (Tekneye göre yerel açı)
            local_target_angle = self.find_best_gap_angle()

            # 2. Global açıya çevir
            global_target_angle = curr_yaw + local_target_angle

            # 3. İleriye bir hedef nokta (Waypoint) koy (Örn: 4 metre ileri)
            lookahead_dist = 4.0
            target_x = curr_x + lookahead_dist * math.cos(global_target_angle)
            target_y = curr_y + lookahead_dist * math.sin(global_target_angle)

            # 4. Hedefi Nav2'ye gönder
            self.send_goal(target_x, target_y, global_target_angle)

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Yaw to Quaternion dönüşümü
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Yeni Waypoint Gönderildi: X={x:.2f}, Y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerMissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
