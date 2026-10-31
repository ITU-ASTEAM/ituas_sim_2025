#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node as RosNode
from geometry_msgs.msg import Twist

from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double

class CmdVelToThrust(RosNode):
    def __init__(self):
        super().__init__('cmd_vel_to_thrust')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.gz_node = GzNode()
        self.left_pub  = self.gz_node.advertise("/model/boat/joint/leftmotor_link_joint/cmd_thrust", Double)
        self.right_pub = self.gz_node.advertise("/model/boat/joint/rightmotor_link_joint/cmd_thrust", Double)

        # --- PARAMETRELER ---
        self.wheel_sep      = 0.52   # motor arası mesafe (m)
        self.angular_mult   = 100.0   # dönüş çarpanı
        self.max_thrust     = 50.0   # maksimum thrust (N)

        # Non-linear thrust: düşük hızda daha fazla kuvvet
        # thrust = lin_vel * linear_mult + dead_band_boost
        self.linear_mult    = 30.0   # <<< 15→30: genel çarpan artırıldı
        self.dead_band_vel  = 0.3    # bu hızın altında extra boost ver
        self.dead_band_boost = 8.0   # extra boost (N) — su direncini yenmek için

    def vel_to_thrust(self, lin_vel):
        """
        Non-linear dönüşüm:
        - Yüksek hız: normal linear_mult
        - Düşük hız (< dead_band_vel): extra boost ekle
        """
        thrust = lin_vel * self.linear_mult

        # Dead-band boost: çok yavaş hızda tekne durmaya başlarsa iterek devam et
        if abs(lin_vel) > 0.01 and abs(lin_vel) < self.dead_band_vel:
            # ne kadar yavaşsa o kadar fazla boost
            boost_factor = 1.0 - (abs(lin_vel) / self.dead_band_vel)
            extra = math.copysign(self.dead_band_boost * boost_factor, lin_vel)
            thrust += extra

        return thrust

    def cmd_vel_callback(self, msg):
        lin = msg.linear.x
        ang = msg.angular.z

        # Lineer thrust (non-linear)
        lin_thrust = self.vel_to_thrust(lin)

        # Diferansiyel dönüş
        turn_diff = ang * self.angular_mult * self.wheel_sep / 2.0

        left_thrust  = -(lin_thrust - turn_diff)
        right_thrust = -(lin_thrust + turn_diff)

        # Clamp
        left_thrust  = max(-self.max_thrust, min(self.max_thrust, left_thrust))
        right_thrust = max(-self.max_thrust, min(self.max_thrust, right_thrust))

        msg_l = Double()
        msg_l.data = left_thrust
        msg_r = Double()
        msg_r.data = right_thrust

        self.left_pub.publish(msg_l)
        self.right_pub.publish(msg_r)

        self.get_logger().info(
            f"lin={lin:.2f} ang={ang:.2f} | "
            f"lin_thrust={lin_thrust:.1f}N | "
            f"Sol: {left_thrust:.1f} N | Sağ: {right_thrust:.1f} N"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThrust()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
