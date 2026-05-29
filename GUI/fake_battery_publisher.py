#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import math
import time

class FakeBatteryPublisher(Node):
    def __init__(self):
        super().__init__('fake_battery_publisher')

        self.declare_parameter('initial_percentage', 0.85)
        self.declare_parameter('drain_rate', 0.0001)
        self.declare_parameter('voltage', 24.0)
        self.declare_parameter('publish_hz', 1.0)

        self.percentage = self.get_parameter('initial_percentage').value
        self.drain_rate = self.get_parameter('drain_rate').value
        self.voltage    = self.get_parameter('voltage').value
        hz              = self.get_parameter('publish_hz').value

        self.pub = self.create_publisher(BatteryState, '/battery', 10)
        self.timer = self.create_timer(1.0 / hz, self.publish_battery)
        self.get_logger().info(f'Fake battery publisher started — {self.percentage*100:.0f}% @ {self.voltage}V')

    def publish_battery(self):
        self.percentage = max(0.0, self.percentage - self.drain_rate)

        msg = BatteryState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.voltage    = self.voltage
        msg.percentage = self.percentage
        msg.current    = -2.5
        msg.charge     = float('nan')
        msg.capacity   = float('nan')
        msg.design_capacity = float('nan')
        msg.power_supply_status   = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health   = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True

        self.pub.publish(msg)

        pct = self.percentage * 100
        if pct < 20:
            self.get_logger().warn(f'LOW BATTERY: {pct:.1f}%', throttle_duration_sec=30)
        else:
            self.get_logger().info(f'Battery: {pct:.1f}% | {self.voltage}V', throttle_duration_sec=10)

def main(args=None):
    rclpy.init(args=args)
    node = FakeBatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
