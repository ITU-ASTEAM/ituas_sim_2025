#!/usr/bin/env python3
"""
Velodyne VLP-16 → Gerçek 3D Costmap — HEPSİNİ KAYDET
/velodyne_points → PGM + YAML + PNG (3D görsel) + PCD + CSV + NPY

TF (ayrı terminal):
  ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
    --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id velodyne

Çalıştır:
  python3 lidar_3d_costmap.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import os
import struct
import zlib
from datetime import datetime

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

# ════════════════════════════════════════════════
#  AYARLAR
# ════════════════════════════════════════════════
RESOLUTION       = 0.10    # XY çözünürlük (metre/hücre)
Z_RESOLUTION     = 0.20    # Z katman kalınlığı
MAP_WIDTH_M      = 20.0
MAP_HEIGHT_M     = 20.0
Z_MIN            = -0.5
Z_MAX            =  4.0

OBSTACLE_THRESH  = 1
INFLATION_RADIUS = 2
MAX_ACCUMULATE   = 50      # kaç frame biriktir (~5 sn @ 10Hz)

FIXED_FRAME      = "map"
SAVE_DIR         = os.path.expanduser("~/Desktop/costmaps")
SAVE_EVERY_N     = 40      # kaç frame'de bir kaydet
PNG_SCALE        = 5       # PNG'de her hücre kaç piksel (büyük = daha güzel görsel)
# ════════════════════════════════════════════════

Z_LAYERS = int((Z_MAX - Z_MIN) / Z_RESOLUTION)


# ──────────────────────────────────────────────
#  YARDIMCI FONKSİYONLAR
# ──────────────────────────────────────────────

from sensor_msgs_py import point_cloud2

def pc2_to_xyz(msg):
    points = []
    for p in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True):
        points.append([p[0], p[1], p[2]])

    if len(points) == 0:
        return None

    return np.array(points, dtype=np.float32)


def build_voxel_grid(pts, rows, cols, ox, oy, res):
    """3D voxel grid → shape (Z_LAYERS, rows, cols)"""
    voxels = np.zeros((Z_LAYERS, rows, cols), dtype=np.int16)
    cx = ((pts[:, 0] - ox)    / res).astype(int)
    cy = ((pts[:, 1] - oy)    / res).astype(int)
    cz = ((pts[:, 2] - Z_MIN) / Z_RESOLUTION).astype(int)
    valid = (cx>=0)&(cx<cols)&(cy>=0)&(cy<rows)&(cz>=0)&(cz<Z_LAYERS)
    np.add.at(voxels, (cz[valid], cy[valid], cx[valid]), 1)
    return voxels


def voxel_to_2d(voxels):
    """3D voxel → 2D OccupancyGrid (herhangi Z'de engel varsa işaretle)"""
    occupied = (voxels >= OBSTACLE_THRESH).any(axis=0)
    grid = np.zeros(occupied.shape, dtype=np.int8)
    grid[occupied] = 100
    return grid


def inflate(grid, radius):
    if radius <= 0:
        return grid.copy()
    binary = (grid == 100).astype(np.uint8)
    row_inf = np.zeros_like(binary)
    for dr in range(-radius, radius + 1):
        s = np.roll(binary, dr, axis=0)
        if dr < 0: s[dr:, :] = 0
        elif dr > 0: s[:dr, :] = 0
        row_inf = np.maximum(row_inf, s)
    col_inf = np.zeros_like(row_inf)
    for dc in range(-radius, radius + 1):
        s = np.roll(row_inf, dc, axis=1)
        if dc < 0: s[:, dc:] = 0
        elif dc > 0: s[:, :dc] = 0
        col_inf = np.maximum(col_inf, s)
    return (col_inf * 100).astype(np.int8)


def z_to_rgb(z_norm):
    """Z yüksekliği → rainbow RGB (mavi=alçak → yeşil → kırmızı=yüksek)"""
    t = np.clip(z_norm, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(t * 4 - 3), 0, 1)
    g = np.clip(1.5 - np.abs(t * 4 - 2), 0, 1)
    b = np.clip(1.5 - np.abs(t * 4 - 1), 0, 1)
    return r, g, b


# ──────────────────────────────────────────────
#  KAYIT FONKSİYONLARI
# ──────────────────────────────────────────────

def write_png(rgb_array, path):
    """Saf Python PNG yazar — dış kütüphane gerektirmez"""
    rows, cols = rgb_array.shape[:2]
    raw = b''.join(b'\x00' + rgb_array[r].tobytes() for r in range(rows))
    comp = zlib.compress(raw, 6)

    def chunk(ct, data):
        c = ct + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', zlib.crc32(c) & 0xffffffff)

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', cols, rows, 8, 2, 0, 0, 0)))
        f.write(chunk(b'IDAT', comp))
        f.write(chunk(b'IEND', b''))


def save_pgm_yaml(grid, res, ox, oy, base, ts):
    """Nav2 uyumlu PGM + YAML"""
    rows, cols = grid.shape
    pgm_path  = base + ".pgm"
    yaml_path = base + ".yaml"

    img = np.where(grid == 100, 0, 254).astype(np.uint8)
    with open(pgm_path, 'wb') as f:
        f.write(f"P5\n{cols} {rows}\n255\n".encode())
        f.write(img.tobytes())

    with open(yaml_path, 'w') as f:
        f.write(f"image: costmap_{ts}.pgm\n"
                f"resolution: {res}\n"
                f"origin: [{ox}, {oy}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n")

    return pgm_path, yaml_path


def save_png_visual(pts, grid, res, ox, oy, base):
    """
    RViz benzeri renkli 3D top-down görsel PNG.
    Koyu arka plan, noktalar Z'ye göre rainbow renkli, engeller parlak beyaz.
    """
    rows, cols = grid.shape
    S = PNG_SCALE
    H, W = rows * S, cols * S

    # Koyu arka plan
    rgb = np.full((H, W, 3), 18, dtype=np.uint8)

    # Grid çizgisi (çok ince, koyu mavi-gri)
    rgb[::S, :] = [30, 35, 45]
    rgb[:, ::S] = [30, 35, 45]

    # Noktaları Z rainbow ile çiz
    if len(pts) > 0:
        cx  = ((pts[:, 0] - ox) / res).astype(int)
        cy  = ((pts[:, 1] - oy) / res).astype(int)
        z_n = np.clip((pts[:, 2] - Z_MIN) / max(Z_MAX - Z_MIN, 0.01), 0, 1)
        r_c, g_c, b_c = z_to_rgb(z_n)
        valid = (cx >= 0) & (cx < cols) & (cy >= 0) & (cy < rows)
        idxs  = np.where(valid)[0]
        for i in idxs:
            px = int(cx[i]) * S
            py = int(cy[i]) * S
            rgb[py:py+S, px:px+S, 0] = int(r_c[i] * 255)
            rgb[py:py+S, px:px+S, 1] = int(g_c[i] * 255)
            rgb[py:py+S, px:px+S, 2] = int(b_c[i] * 255)

    # Engel hücreleri — parlak beyaz kenarlık
    obs_rows, obs_cols = np.where(grid == 100)
    for r, c in zip(obs_rows, obs_cols):
        px, py = c * S, r * S
        # Sadece kenara beyaz çizgi (içi nokta rengi kalsın)
        rgb[py,       px:px+S] = [230, 230, 230]
        rgb[py+S-1,   px:px+S] = [230, 230, 230]
        rgb[py:py+S,  px     ] = [230, 230, 230]
        rgb[py:py+S,  px+S-1 ] = [230, 230, 230]

    png_path = base + "_visual.png"
    write_png(rgb, png_path)
    return png_path


def save_pcd(pts, base):
    """
    ASCII PCD dosyası — CloudCompare, MeshLab, Open3D ile açılır.
    Noktalar X Y Z + RGB (Z'ye göre renkli) içerir.
    """
    pcd_path = base + ".pcd"
    n = len(pts)
    z_n  = np.clip((pts[:,2]-Z_MIN)/max(Z_MAX-Z_MIN,0.01), 0, 1)
    r_c, g_c, b_c = z_to_rgb(z_n)
    r_i = (r_c * 255).astype(np.uint32)
    g_i = (g_c * 255).astype(np.uint32)
    b_i = (b_c * 255).astype(np.uint32)
    rgb_packed = (r_i << 16) | (g_i << 8) | b_i

    with open(pcd_path, 'w') as f:
        f.write("# PCD v0.7 - Velodyne VLP-16 3D Costmap\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z rgb\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F U\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for i in range(n):
            f.write(f"{pts[i,0]:.4f} {pts[i,1]:.4f} {pts[i,2]:.4f} {rgb_packed[i]}\n")

    return pcd_path


def save_csv(pts, base):
    """CSV — Excel, Python, MATLAB ile açılır"""
    csv_path = base + ".csv"
    z_n  = np.clip((pts[:,2]-Z_MIN)/max(Z_MAX-Z_MIN,0.01), 0, 1)
    r_c, g_c, b_c = z_to_rgb(z_n)
    with open(csv_path, 'w') as f:
        f.write("x,y,z,r,g,b\n")
        for i in range(len(pts)):
            f.write(f"{pts[i,0]:.4f},{pts[i,1]:.4f},{pts[i,2]:.4f},"
                    f"{r_c[i]:.3f},{g_c[i]:.3f},{b_c[i]:.3f}\n")
    return csv_path


def save_npy(voxels, pts, base):
    """Ham veri — Python ile yükle: np.load(...)"""
    vox_path = base + "_voxels.npy"
    pts_path = base + "_points.npy"
    np.save(vox_path, voxels)
    np.save(pts_path, pts)
    return vox_path, pts_path


def save_all(voxels, pts, grid_2d, res, ox, oy):
    """Tüm formatları masaüstüne kaydeder."""
    os.makedirs(SAVE_DIR, exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(SAVE_DIR, f"costmap_{ts}")

    pgm,  yaml      = save_pgm_yaml(grid_2d, res, ox, oy, base, ts)
    png              = save_png_visual(pts, grid_2d, res, ox, oy, base)
    pcd              = save_pcd(pts, base)
    csv              = save_csv(pts, base)
    vox_npy, pts_npy = save_npy(voxels, pts, base)

    return {
        "PGM (Nav2)":    pgm,
        "YAML (Nav2)":   yaml,
        "PNG (görsel)":  png,
        "PCD (3D)":      pcd,
        "CSV":           csv,
        "NPY voxels":    vox_npy,
        "NPY points":    pts_npy,
    }


# ──────────────────────────────────────────────
#  ROS2 NODE
# ──────────────────────────────────────────────

class Costmap3DNode(Node):
    def __init__(self):
        super().__init__('costmap_3d_node')

        sq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, depth=5)
        rq = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.sub    = self.create_subscription(PointCloud2, '/velodyne_points', self.cb, sq)
        self.pub_og = self.create_publisher(OccupancyGrid, '/costmap_3d',       rq)
        self.pub_mk = self.create_publisher(MarkerArray,   '/costmap_markers',  10)
        self.pub_pc = self.create_publisher(PointCloud2,   '/costmap_filtered', 10)

        self.res   = RESOLUTION
        self.cols  = int(MAP_WIDTH_M  / self.res)
        self.rows  = int(MAP_HEIGHT_M / self.res)
        self.ox    = -MAP_WIDTH_M  / 2.0
        self.oy    = -MAP_HEIGHT_M / 2.0
        self.accum = []
        self.cnt   = 0

        self.get_logger().info(
            f"\n  ✅ 3D Costmap Node başlatıldı"
            f"\n  📡 Topic   : /velodyne_points"
            f"\n  🧊 Voxel   : {self.cols}x{self.rows}x{Z_LAYERS} @ {self.res}m / {Z_RESOLUTION}m"
            f"\n  📁 Kayıt   : {SAVE_DIR}"
            f"\n  💾 Formatlar: PGM · YAML · PNG · PCD · CSV · NPY"
        )

    def cb(self, msg: PointCloud2):
        pts = pc2_to_xyz(msg)
        if pts is None or len(pts) == 0:
            return

        mask = (
	        np.isfinite(pts).all(axis=1) &
	        (np.abs(pts[:,0]) < 100) &
	        (np.abs(pts[:,1]) < 100) &
	        (np.abs(pts[:,2]) < 50)
	    )
        pts = pts[mask]
        if len(pts) == 0:
            return

        self.accum.append(pts)
        if len(self.accum) > MAX_ACCUMULATE:
            self.accum.pop(0)
        all_pts = np.vstack(self.accum)
        stamp   = msg.header.stamp

        # Filtrelenmiş PC2 yayınla
        self.pub_pc.publish(self._pc2_msg(all_pts, stamp))

        # 3D voxel grid
        voxels   = build_voxel_grid(all_pts, self.rows, self.cols, self.ox, self.oy, self.res)
        grid_2d  = voxel_to_2d(voxels)
        inflated = inflate(grid_2d, INFLATION_RADIUS)

        # OccupancyGrid yayınla
        self.pub_og.publish(self._og_msg(inflated, stamp))

        # 3D Marker yayınla (RViz'de rainbow voxel küpleri)
        self.pub_mk.publish(self._mk_msg(all_pts, voxels, stamp))

        # Kayıt
        self.cnt += 1
        if self.cnt % SAVE_EVERY_N == 0:
            saved = save_all(voxels, all_pts, inflated, self.res, self.ox, self.oy)
            self.get_logger().info(f"\n  💾 Kaydedildi → {SAVE_DIR}/")
            for fmt, path in saved.items():
                self.get_logger().info(f"     {fmt:15s}: {os.path.basename(path)}")

    # ── Yardımcı mesaj oluşturucular ──────────

    def _pc2_msg(self, pts, stamp):
        msg = PointCloud2()
        msg.header.frame_id = FIXED_FRAME
        msg.header.stamp    = stamp
        msg.height = 1; msg.width = len(pts)
        msg.is_dense = False; msg.is_bigendian = False
        flds = []
        for i, n in enumerate(['x','y','z']):
            f = PointField(); f.name=n; f.offset=i*4
            f.datatype=PointField.FLOAT32; f.count=1; flds.append(f)
        msg.fields     = flds
        msg.point_step = 12
        msg.row_step   = 12 * len(pts)
        msg.data       = pts.astype(np.float32).tobytes()
        return msg

    def _og_msg(self, grid, stamp):
        og = OccupancyGrid()
        og.header.frame_id = FIXED_FRAME
        og.header.stamp    = stamp
        og.info.resolution = self.res
        og.info.width      = self.cols
        og.info.height     = self.rows
        og.info.origin.position.x = self.ox
        og.info.origin.position.y = self.oy
        og.info.origin.position.z = 0.0
        og.data = grid.flatten().tolist()
        return og

    def _mk_msg(self, pts, voxels, stamp):
        ma = MarkerArray()

        # Nokta bulutu — Z rainbow
        m = Marker()
        m.header.frame_id = FIXED_FRAME; m.header.stamp = stamp
        m.ns="pts"; m.id=0; m.type=Marker.POINTS; m.action=Marker.ADD
        m.scale.x = m.scale.y = 0.06

        z_n = np.clip((pts[:,2]-Z_MIN)/max(Z_MAX-Z_MIN,0.01), 0, 1)
        r_c, g_c, b_c = z_to_rgb(z_n)
        idx = np.random.choice(len(pts), min(len(pts), 15000), replace=False)
        for i in idx:
            p = Point(); p.x,p.y,p.z = float(pts[i,0]),float(pts[i,1]),float(pts[i,2])
            c = ColorRGBA()
            c.r,c.g,c.b,c.a = float(r_c[i]),float(g_c[i]),float(b_c[i]),0.95
            m.points.append(p); m.colors.append(c)
        ma.markers.append(m)

        # 3D voxel küpler — Z'ye göre renkli
        m2 = Marker()
        m2.header.frame_id = FIXED_FRAME; m2.header.stamp = stamp
        m2.ns="voxels"; m2.id=1; m2.type=Marker.CUBE_LIST; m2.action=Marker.ADD
        m2.scale.x = m2.scale.y = self.res * 0.9
        m2.scale.z = Z_RESOLUTION * 0.9
        m2.color.a = 1.0

        occ = np.argwhere(voxels >= OBSTACLE_THRESH)
        for (zi, ri, ci) in occ:
            p = Point()
            p.x = self.ox + ci * self.res       + self.res       / 2
            p.y = self.oy + ri * self.res       + self.res       / 2
            p.z = Z_MIN   + zi * Z_RESOLUTION   + Z_RESOLUTION   / 2
            m2.points.append(p)
            z_norm = zi / max(Z_LAYERS - 1, 1)
            rc, gc, bc = z_to_rgb(np.array([z_norm]))
            c = ColorRGBA()
            c.r,c.g,c.b,c.a = float(rc[0]),float(gc[0]),float(bc[0]),0.75
            m2.colors.append(c)
        ma.markers.append(m2)

        return ma


# ──────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = Costmap3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
