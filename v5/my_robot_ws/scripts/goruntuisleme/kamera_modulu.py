#!/usr/bin/env python3
import numpy as np
import cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

class KameraModulu:
    def __init__(self, topic_name):
        self.node = Node()
        self.frame = None
        self.topic_name = topic_name
        self.is_depth = "depth" in topic_name
        
        print(f"[{self.topic_name}] Modül Başlatıldı (Derinlik Modu: {self.is_depth})")
        self.node.subscribe(Image, self.topic_name, self.callback)

    def callback(self, msg):
        try:
            width = msg.width
            height = msg.height
            
            # --- DERİNLİK VERİSİ ---
            if self.is_depth:
                # Gazebo depth verisini bazen byte, bazen float buffer olarak yollar
                data = np.frombuffer(msg.data, dtype=np.float32)
                # Boyut kontrolü
                if data.size == width * height:
                    self.frame = data.reshape((height, width))
                return

            # --- RGB VERİSİ ---
            data = np.frombuffer(msg.data, dtype=np.uint8)
            channels = len(data) // (width * height)
            
            if channels not in [3, 4]: return # Hatalı veri koruması

            img = data.reshape((height, width, channels))
            
            # Renk düzeltme
            if channels == 4:
                self.frame = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            else:
                self.frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                
        except Exception:
            pass

    def get_frame(self):
        if self.frame is not None:
            return True, self.frame
        return False, None
