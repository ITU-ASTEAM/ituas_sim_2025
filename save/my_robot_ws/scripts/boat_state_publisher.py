#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class BoatStatePublisher(Node):
    def __init__(self):
        super().__init__('boat_state_publisher')
        # Gazebo'dan gelen ham pose verisini dinle
        self.sub = self.create_subscription(PoseStamped, '/model/boat/pose', self.callback, 10)
        # Nav2'nin beklediği odom mesajını yayınla
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # TF ağacını (odom -> base_link) oluştur
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('Gemi konum ve aci verisi aktarimi aktif.')

    def callback(self, msg):
        # 1. ODOMETRY YAYINI
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = msg.pose # Koordinat ve Açı (Quaternion)
        self.odom_pub.publish(odom)

        # 2. TF DÖNÜŞÜMÜ (Nav2'nin kalbi burası)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(BoatStatePublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

