#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

DIRECTIONS = [
    "N", "NNE", "NE", "ENE",
    "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW",
    "W", "WNW", "NW", "NNW"
]

def quaternion_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw_rad)
    if yaw_deg < 0:
        yaw_deg += 360.0
    return yaw_deg

def deg_to_cardinal(deg):
    idx = round(deg / 22.5) % 16
    return DIRECTIONS[idx]

class VesselStatusPublisher(Node):
    def __init__(self):
        super().__init__('vessel_status_publisher')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(String, '/vessel_status', 10)
        self.get_logger().info('Vessel status publisher started')

    def odom_callback(self, msg):
        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        ow = msg.pose.pose.orientation.w
        yaw_deg = quaternion_to_yaw_deg(ox, oy, oz, ow)
        cardinal = deg_to_cardinal(yaw_deg)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed_knots = math.sqrt(vx**2 + vy**2) * 1.94384

        status = (
            f"Heading : {yaw_deg:05.1f}°  {cardinal}\n"
            f"Speed   : {speed_knots:.2f} kn"
        )

        out = String()
        out.data = status
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = VesselStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
