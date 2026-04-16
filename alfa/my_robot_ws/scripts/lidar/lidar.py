#!/usr/bin/env python3
"""
LiDAR + Pozisyon Görselleştirme (SADECE GAZEBO TRANSPORT)
Bridge gerektirmez!
"""

import math
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.laserscan_pb2 import LaserScan

class LidarPozisyonVisualizer:
    def __init__(self):
        self.node = Node()
        
        # Pozisyon değişkenleri
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.pose_ready = False
        
        # LiDAR değişkenleri
        self.ranges = []
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.lidar_ready = False
        
        # Subscriptions
        pose_topic = "/world/ocean_world/model/boat/pose"
        lidar_topic = "/lidar"
        
        print(f"[System] Pozisyon topic: {pose_topic}")
        print(f"[System] LiDAR topic: {lidar_topic}")
        
        # Pose subscription
        if not self.node.subscribe(Pose_V, pose_topic, self.pose_callback_v):
            print("[ERROR] Pose_V subscribe failed!")
        if not self.node.subscribe(Pose, pose_topic, self.pose_callback):
            print("[ERROR] Pose subscribe failed!")
        
        # LiDAR subscription
        if not self.node.subscribe(LaserScan, lidar_topic, self.lidar_callback):
            print("[ERROR] LiDAR subscribe failed!")
        else:
            print("[OK] LiDAR subscribed!")
        
        # Matplotlib setup
        self.fig = plt.figure(figsize=(14, 7))
        
        # 1. Polar LiDAR
        self.ax1 = plt.subplot(131, projection='polar')
        self.ax1.set_theta_zero_location('N')
        self.ax1.set_theta_direction(-1)
        self.ax1.set_ylim(0, 30)
        self.ax1.set_title('LiDAR Radar View', fontsize=12, fontweight='bold')
        self.line1, = self.ax1.plot([], [], 'b.', markersize=2, label='Scan')
        self.obstacle_line, = self.ax1.plot([], [], 'ro', markersize=6, label='Close (<5m)')
        self.ax1.legend()
        
        # 2. World Map
        self.ax2 = plt.subplot(132)
        self.ax2.set_xlim(-50, 50)
        self.ax2.set_ylim(-50, 50)
        self.ax2.set_xlabel('X (m)')
        self.ax2.set_ylabel('Y (m)')
        self.ax2.set_title('World Map (Top-Down)', fontsize=12, fontweight='bold')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_aspect('equal')
        
        self.scatter = self.ax2.scatter([], [], c='blue', s=1, alpha=0.5, label='LiDAR')
        self.obstacle_scatter = self.ax2.scatter([], [], c='red', s=40, marker='X', label='Obstacles')
        self.boat_arrow = None
        self.boat_path, = self.ax2.plot([], [], 'g--', linewidth=1, alpha=0.5, label='Path')
        self.ax2.legend()
        
        self.path_x = []
        self.path_y = []
        
        # 3. Info Panel
        self.ax3 = plt.subplot(133)
        self.ax3.axis('off')
        self.info_text = self.ax3.text(0.1, 0.9, '', fontsize=10, 
                                       verticalalignment='top', family='monospace')
        
        # Animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        
        print("[System] Visualization started! Waiting for data...")

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_callback_v(self, msg):
        if len(msg.pose) == 0:
            return
        p = msg.pose[0]
        self.x = p.position.x
        self.y = p.position.y
        self.yaw = self.quaternion_to_yaw(
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w
        )
        if not self.pose_ready:
            print(f"[OK] Pose data received! X={self.x:.2f}, Y={self.y:.2f}")
        self.pose_ready = True

    def pose_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.yaw = self.quaternion_to_yaw(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )
        if not self.pose_ready:
            print(f"[OK] Pose data received! X={self.x:.2f}, Y={self.y:.2f}")
        self.pose_ready = True

    def lidar_callback(self, msg):
        # LaserScan mesajından veriyi al
        self.ranges = np.array(list(msg.ranges))
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        
        # Angle increment hesapla
        if len(self.ranges) > 1:
            self.angle_increment = (self.angle_max - self.angle_min) / (len(self.ranges) - 1)
        
        if not self.lidar_ready:
            print(f"[OK] LiDAR data received! {len(self.ranges)} points")
        self.lidar_ready = True

    def update_plot(self, frame):
        info_lines = []
        info_lines.append("=" * 40)
        info_lines.append("  BOAT STATUS")
        info_lines.append("=" * 40)
        
        # POSE UPDATE
        if self.pose_ready:
            if self.boat_arrow:
                self.boat_arrow.remove()
            
            c, s = np.cos(self.yaw), np.sin(self.yaw)
            self.boat_arrow = self.ax2.arrow(
                self.x, self.y, 3*c, 3*s,
                head_width=1.5, head_length=1.0,
                fc='green', ec='darkgreen', linewidth=2, zorder=5
            )
            
            self.path_x.append(self.x)
            self.path_y.append(self.y)
            if len(self.path_x) > 1000:
                self.path_x.pop(0)
                self.path_y.pop(0)
            self.boat_path.set_data(self.path_x, self.path_y)
            
            self.ax2.set_xlim(self.x - 30, self.x + 30)
            self.ax2.set_ylim(self.y - 30, self.y + 30)
            
            info_lines.append(f"Position:")
            info_lines.append(f"  X: {self.x:8.2f} m")
            info_lines.append(f"  Y: {self.y:8.2f} m")
            info_lines.append(f"  Yaw: {np.degrees(self.yaw):6.1f}°")
            info_lines.append("")
        else:
            info_lines.append("Position: WAITING...")
            info_lines.append("")
        
        # LIDAR UPDATE
        if self.lidar_ready and len(self.ranges) > 0:
            angles = np.linspace(self.angle_min, self.angle_max, len(self.ranges))
            
            valid_mask = ~np.isinf(self.ranges)
            valid_ranges = self.ranges[valid_mask]
            valid_angles = angles[valid_mask]
            
            if len(valid_ranges) > 0:
                # Polar plot
                self.line1.set_data(valid_angles, valid_ranges)
                
                close_mask = valid_ranges < 5.0
                if np.any(close_mask):
                    self.obstacle_line.set_data(valid_angles[close_mask], valid_ranges[close_mask])
                else:
                    self.obstacle_line.set_data([], [])
                
                # World coordinates
                lidar_offset = 0.5
                x_robot = valid_ranges * np.sin(valid_angles) + lidar_offset
                y_robot = valid_ranges * np.cos(valid_angles)
                
                if self.pose_ready:
                    c, s = np.cos(self.yaw), np.sin(self.yaw)
                    x_world = self.x + (x_robot * c - y_robot * s)
                    y_world = self.y + (x_robot * s + y_robot * c)
                    self.scatter.set_offsets(np.c_[x_world, y_world])
                    
                    if np.any(close_mask):
                        x_close_robot = valid_ranges[close_mask] * np.sin(valid_angles[close_mask]) + lidar_offset
                        y_close_robot = valid_ranges[close_mask] * np.cos(valid_angles[close_mask])
                        x_close_world = self.x + (x_close_robot * c - y_close_robot * s)
                        y_close_world = self.y + (x_close_robot * s + y_close_robot * c)
                        self.obstacle_scatter.set_offsets(np.c_[x_close_world, y_close_world])
                    else:
                        self.obstacle_scatter.set_offsets(np.empty((0, 2)))
                
                # Stats
                min_dist = np.min(valid_ranges)
                max_dist = np.max(valid_ranges)
                avg_dist = np.mean(valid_ranges)
                obstacle_count = np.sum(close_mask)
                
                min_idx = np.argmin(valid_ranges)
                min_angle_deg = np.degrees(valid_angles[min_idx])
                
                info_lines.append(f"LiDAR:")
                info_lines.append(f"  Points: {len(valid_ranges)}")
                info_lines.append(f"  Min: {min_dist:6.2f} m @ {min_angle_deg:6.1f}°")
                info_lines.append(f"  Avg: {avg_dist:6.2f} m")
                info_lines.append(f"  Max: {max_dist:6.2f} m")
                info_lines.append("")
                info_lines.append(f"Obstacles (<5m): {obstacle_count}")
                
                if obstacle_count > 0:
                    info_lines.append(f"  WARNING: {obstacle_count} close!")
        else:
            info_lines.append("LiDAR: WAITING...")
        
        info_lines.append("")
        info_lines.append("=" * 40)
        info_lines.append(f"Pose: {'OK' if self.pose_ready else 'NO'}")
        info_lines.append(f"LiDAR: {'OK' if self.lidar_ready else 'NO'}")
        info_lines.append("=" * 40)
        
        self.info_text.set_text('\n'.join(info_lines))

def main():
    print("=" * 60)
    print("LIDAR + POZISYON VISUALIZER (Gazebo Transport)")
    print("=" * 60)
    
    viz = LidarPozisyonVisualizer()
    plt.show(block=False)
    
    try:
        while True:
            time.sleep(0.05)
            plt.pause(0.01)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        plt.close('all')

if __name__ == '__main__':
    main()
