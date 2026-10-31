#!/usr/bin/env python3
"""
Tennis Ball Costmap Node
- /mistik_points dinler, birikimli costmap üretir
- 260x280 piksel çıktı, 15m = 260px
- Taze tespit: mavi → beyaz
- Eski/soluk tespit: sarı (decay rengi)
- Robot: kırmızı nokta (merkez alt)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import cv2
import os
import math
import threading
from datetime import datetime


# ════════════════════════════════════════════
#  AYARLAR
# ════════════════════════════════════════════
MAP_W_PX     = 260      # Çıktı genişliği (piksel)
MAP_H_PX     = 280      # Çıktı yüksekliği (piksel)
MAP_RANGE_M  = 15.0     # MAP_W_PX kaç metreye karşılık gelir
CELL_SIZE    = 4        # Her nokta kaç px (kare)
DECAY_FACTOR = 0.994    # İz sönümü — düşürünce daha hızlı solar
HIT_INC      = 0.5      # Her tespitte "tazelik" artışı
SAVE_SEC     = 6.0      # Kayıt aralığı saniye (6s = dk'da 10x)
SAVE_DIR     = os.path.expanduser("~/costmaps")

# Tazelik eşiği: bu değerin altındaki grid hücreleri "eski" sayılır
# Yeni tespit → tazelik yüksek → mavi/beyaz
# Soluk tespit → tazelik düşük → sarı
FRESH_THRESH = 0.35
# ════════════════════════════════════════════

PX_PER_M = MAP_W_PX / MAP_RANGE_M
GRID_W   = MAP_W_PX // CELL_SIZE
GRID_H   = MAP_H_PX // CELL_SIZE


def val_to_color(v: float) -> tuple | None:
    """
    Tazelik değerine göre renk:
      v < 0.05              → görünmez (siyah)
      0.05 < v < FRESH_THRESH → sarı tonları (eski iz)
      v > FRESH_THRESH      → mavi → cyan → beyaz (taze)
    BGR döner.
    """
    if v < 0.05:
        return None

    if v < FRESH_THRESH:
        # Eski iz: koyu sarıdan parlak sarıya
        t = v / FRESH_THRESH  # 0→1
        b = 0
        g = int(160 + 95 * t)
        r = int(160 + 95 * t)
        return (b, g, r)   # sarı (BGR: 0, ~200, ~200)

    else:
        # Taze: mavi → cyan → beyaz
        t = (v - FRESH_THRESH) / (1.0 - FRESH_THRESH)  # 0→1
        if t < 0.4:
            tt = t / 0.4
            return (int(180 + 75 * tt), 0, 0)           # koyu mavi → mavi
        elif t < 0.7:
            tt = (t - 0.4) / 0.3
            return (255, int(220 * tt), 0)               # mavi → cyan
        elif t < 0.9:
            tt = (t - 0.7) / 0.2
            return (255, 255, int(200 * tt))             # cyan → beyaz
        else:
            return (255, 255, 255)                       # beyaz


def world_to_grid(x_m: float, z_m: float, yaw_deg: float):
    """Metre → grid hücresi. Orijin alt-orta."""
    yaw_rad = math.radians(yaw_deg)
    wx =  x_m * math.cos(yaw_rad) - z_m * math.sin(yaw_rad)
    wz =  x_m * math.sin(yaw_rad) + z_m * math.cos(yaw_rad)
    gx = int(GRID_W / 2 + wx * PX_PER_M / CELL_SIZE)
    gy = int(GRID_H     - wz * PX_PER_M / CELL_SIZE)
    return gx, gy


def render(grid: np.ndarray) -> np.ndarray:
    """Grid → görüntü."""
    img = np.zeros((MAP_H_PX, MAP_W_PX, 3), dtype=np.uint8)

    for gy in range(GRID_H):
        for gx in range(GRID_W):
            color = val_to_color(grid[gy, gx])
            if color is None:
                continue
            x0 = gx * CELL_SIZE + 1
            y0 = gy * CELL_SIZE + 1
            x1 = x0 + CELL_SIZE - 2
            y1 = y0 + CELL_SIZE - 2
            cv2.rectangle(img, (x0, y0), (x1, y1), color, -1)

    # Robot: kırmızı daire, alt-orta
    rpx = MAP_W_PX // 2
    rpy = MAP_H_PX - CELL_SIZE * 2
    cv2.circle(img, (rpx, rpy), CELL_SIZE, (0, 0, 220), -1)   # kırmızı dolu daire
    cv2.circle(img, (rpx, rpy), CELL_SIZE, (0, 0, 255), 1)    # kenar

    return img


class TennisCostmapNode(Node):
    def __init__(self):
        super().__init__('tennis_costmap_node')

        self.grid = np.zeros((GRID_H, GRID_W), dtype=np.float32)
        self.lock = threading.Lock()
        self.current_yaw_deg = 0.0
        self._ask_yaw_once()

        os.makedirs(SAVE_DIR, exist_ok=True)
        self.get_logger().info(
            f"Başlatıldı | {MAP_W_PX}x{MAP_H_PX}px | "
            f"{MAP_RANGE_M}m={MAP_W_PX}px | Grid:{GRID_W}x{GRID_H}"
        )

        self.create_subscription(PointCloud2, '/mistik_points', self.pc_callback, 10)
        self.create_timer(SAVE_SEC, self.save_costmap)
        self.create_timer(0.5, self.apply_decay)
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info("Açı değiştirmek için terminale derece yaz + Enter")

    def _ask_yaw_once(self):
        try:
            v = input("\n>>> YAW açısı (derece, 0=düz ileri): ").strip()
            self.current_yaw_deg = float(v)
        except Exception:
            self.current_yaw_deg = 0.0
        print(f">>> Açı: {self.current_yaw_deg}°\n")

    def _keyboard_loop(self):
        while True:
            try:
                v = input()
                self.current_yaw_deg = float(v)
                self.get_logger().info(f"Açı → {self.current_yaw_deg}°")
            except ValueError:
                print("Sayı gir (örn: 45)")
            except Exception:
                break

    def pc_callback(self, msg: PointCloud2):
        # pcl.py formatı: [z_mesafe, -x_lateral, -y_dikey]
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as e:
            self.get_logger().error(f"PC hata: {e}")
            return

        with self.lock:
            for pt in points:
                z_m =  float(pt[0])
                x_m = -float(pt[1])
                gx, gy = world_to_grid(x_m, z_m, self.current_yaw_deg)
                if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                    # Tespit: tazeliği maksimuma çek
                    self.grid[gy, gx] = min(1.0, self.grid[gy, gx] + HIT_INC)

    def apply_decay(self):
        """Her 0.5s'de tazelik azalır → taze mavi/beyaz → sarıya döner → söner."""
        with self.lock:
            self.grid *= DECAY_FACTOR
            np.clip(self.grid, 0.0, 1.0, out=self.grid)

    def save_costmap(self):
        with self.lock:
            g = self.grid.copy()

        img = render(g)
        fname = os.path.join(
            SAVE_DIR,
            f"costmap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        )
        cv2.imwrite(fname, img)
        self.get_logger().info(f"Kaydedildi → {fname}")


def main(args=None):
    rclpy.init(args=args)
    node = TennisCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()