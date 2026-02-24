#!/usr/bin/env python3
import time
import math
from position import PozisyonModulu

def test_yaw():
    pozisyon = PozisyonModulu(topic_name="/world/ocean_world/model/boat/pose")
    
    print("YAW TEST - Robotu Gazebo'da döndür")
    print("=" * 50)
    
    time.sleep(1)  # Topic'e bağlansın
    
    for i in range(50):
        x, y, yaw = pozisyon.get_pose()
        yaw_deg = math.degrees(yaw)
        
        print(f"Pos: ({x:6.2f}, {y:6.2f})  |  Yaw: {yaw:6.3f} rad = {yaw_deg:7.2f}°  |  Hazır: {pozisyon.hazir}")
        time.sleep(0.2)

if __name__ == "__main__":
    test_yaw()
