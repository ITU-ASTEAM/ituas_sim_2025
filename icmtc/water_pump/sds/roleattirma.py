#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import sys
import select
import termios
import tty

class IndicatorLightNode(Node):
    def __init__(self):
        super().__init__('indicator_light_node')
        self.serial_port = '/dev/ttyACM0'
        
        try:
            self.arduino = serial.Serial(self.serial_port, 115200, timeout=1)
            self.get_logger().info("✅ Arduino bağlı.")
        except:
            self.arduino = None
            self.get_logger().error("❌ Arduino bulunamadı!")
        
        self.get_logger().info("🎮 Kontrol için: 'g' (Guided/Yeşil), 'm' (Manual/Sarı) tuşlarına bas.")
        self.create_timer(0.1, self.keyboard_loop)

    def keyboard_loop(self):
        char = self.get_key()
        if char == 'g':
            if self.arduino: self.arduino.write(b'G')
            self.get_logger().info("🟢 Otonom Mod (G) Aktif")
        elif char == 'm':
            if self.arduino: self.arduino.write(b'M')
            self.get_logger().info("🟡 Manuel Mod (M) Aktif")
        elif char == '\x03': # Ctrl+C'ye basılırsa güvenli çıkış
            sys.exit(0)

    def get_key(self):
        # Terminali anlık okuma moduna alır
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            # Timeout 0.0 diyerek programı kilitlemeden tuş var mı diye bakarız
            if select.select([sys.stdin], [], [], 0.0)[0]:
                return sys.stdin.read(1)
        finally:
            # İşlem bitince terminali bozmamak için eski haline geri getirir
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return None

def main():
    rclpy.init()
    node = IndicatorLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
