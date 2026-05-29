#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage

class GzCameraBridge(Node):
    def __init__(self):
        super().__init__('gz_camera_bridge')

        self.pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)

        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, '/camera/image_raw', self.gz_callback)

        self.get_logger().info('Gz→ROS2 camera bridge started: /camera/image_raw → /oak/rgb/image_raw')

    def gz_callback(self, msg):
        try:
            width  = msg.width
            height = msg.height
            data   = np.frombuffer(msg.data, dtype=np.uint8)
            channels = len(data) // (width * height)

            if channels not in [3, 4]:
                return

            ros_msg = Image()
            ros_msg.header.stamp    = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = 'camera_link'
            ros_msg.height   = height
            ros_msg.width    = width
            ros_msg.step     = width * 3
            ros_msg.encoding = 'rgb8'

            if channels == 4:
                img = data.reshape((height, width, 4))
                ros_msg.data = img[:, :, :3].tobytes()
            else:
                ros_msg.data = data.tobytes()

            self.pub.publish(ros_msg)

        except Exception as e:
            self.get_logger().warn(f'Camera bridge error: {e}', throttle_duration_sec=5)

def main(args=None):
    rclpy.init(args=args)
    node = GzCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
