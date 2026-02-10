#!/usr/bin/env python3
import sys
import numpy as np
import cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

class KameraModulu:
    def __init__(self, topic_name="/camera/image_raw"):
        """
        OAK-D yerine Gazebo topic'ini dinleyen sınıf.
        """
        self.node = Node()
        self.frame = None
        self.topic_name = topic_name
        
        # Gazebo topic'ine abone ol
        print(f"[{self.topic_name}] dinleniyor...")
        self.node.subscribe(Image, self.topic_name, self.callback)
        
        # Bağlantı kurulana kadar bekleme opsiyonu eklenebilir
        # ancak thread bloklamamak için şimdilik boş bırakıyoruz.

    def callback(self, msg):
        try:
            # Gazebo mesajını OpenCV formatına çevir
            width = msg.width
            height = msg.height
            data = np.frombuffer(msg.data, dtype=np.uint8)
            
            # Format dönüşümü (R8G8B8 -> BGR)
            img = data.reshape((height, width, 3))
            self.frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
        except Exception as e:
            print(f"Görüntü alma hatası: {e}")

    def get_frame(self):
        """
        Ana döngüye son alınan kareyi döndürür.
        """
        if self.frame is not None:
            return True, self.frame
        else:
            return False, None

    def release(self):
        # Gazebo transport otomatik temizlenir, özel bir release gerekmez
        pass
