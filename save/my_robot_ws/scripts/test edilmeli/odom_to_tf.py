#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        # /odom topic'ini dinliyoruz
        self.subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10)
        # TF yayıncısını oluşturuyoruz
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('Odom to TF donusturucu basladi. Rasyonel veri bekleniyor...')

    def odom_callback(self, msg):
        t = TransformStamped()
        
        # Zaman damgasını ve referans isimlerini odom mesajından al
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Pozisyon verisini aktar
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Rotasyon (Quaternion) verisini aktar
        t.transform.rotation = msg.pose.pose.orientation
        
        # TF ağacına bas
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

