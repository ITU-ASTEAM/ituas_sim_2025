#!/usr/bin/env python3
import rclpy
from rclpy.node import Node as RosNode
from geometry_msgs.msg import Twist

# Gazebo Transport Kütüphaneleri (Senin çalışan scriptinden)
from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double

class CmdVelToThrust(RosNode):
    def __init__(self):
        super().__init__('cmd_vel_to_thrust')
        
        # ROS 2'den Nav2 Hız Komutunu Dinle
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Doğrudan Gazebo'ya Yayıncı (Bridge İptal)
        self.gz_node = GzNode()
        self.left_pub = self.gz_node.advertise("/model/boat/joint/leftmotor_link_joint/cmd_thrust", Double)
        self.right_pub = self.gz_node.advertise("/model/boat/joint/rightmotor_link_joint/cmd_thrust", Double)
        
        self.wheel_sep = 0.66
        self.thrust_mult = 15.0

    def cmd_vel_callback(self, msg):
        left_vel = msg.linear.x - (msg.angular.z * self.wheel_sep / 2.0)
        right_vel = msg.linear.x + (msg.angular.z * self.wheel_sep / 2.0)
        
        l_thrust = -1.0 * left_vel * self.thrust_mult
        r_thrust = -1.0 * right_vel * self.thrust_mult
        
        # Gazebo Double Mesajlarını Oluştur
        msg_l = Double()
        msg_l.data = l_thrust
        msg_r = Double()
        msg_r.data = r_thrust
        
        # Doğrudan Gazebo'ya Bas
        self.left_pub.publish(msg_l)
        self.right_pub.publish(msg_r)
        
        self.get_logger().info(f"Direct GZ Pub -> Sol: {l_thrust:.2f} N | Sağ: {r_thrust:.2f} N")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThrust()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
