import numpy as np
import trimesh
from PIL import Image, ImageDraw
import yaml
import os

# --- AYARLAR ---
SLICE_HEIGHT_MIN = 0.05   # Zemin gürültüsünü engellemek için
RESOLUTION = 0.05         # 1 piksel = 5cm
BACKGROUND_COLOR = (255, 255, 255, 255) # Arkaplan (Beyaz - Boş)
DEFAULT_OBSTACLE_COLOR = (50, 50, 50, 255) # Renk okunamayan engeller için (Koyu Gri)
# ----------------

def get_scene_geometry(scene):
    geometries = []
    if hasattr(scene, 'graph'):
        for node_name in scene.graph.nodes_geometry:
            transform, geometry_name = scene.graph.get(node_name)
            if geometry_name not in scene.geometry:
                continue
                
            geometry = scene.geometry[geometry_name].copy()
            geometry.apply_transform(transform)
            geometries.append(geometry)
    else:
        # Eğer tek bir mesh ise
        geometries.append(scene)
    
    return geometries

def generate_colored_map(dae_file, output_name="colored_map"):
    print(f"Loading {dae_file}...")
    try:
        # Force 'mesh' to avoid loading as raw scene if possible, handle errors gracefully
        mesh_or_scene = trimesh.load(dae_file, force='mesh') 
    except Exception as e:
        print(f"Standard load failed, trying generic load. Error: {e}")
        mesh_or_scene = trimesh.load(dae_file)
    
    all_meshes = []
    if isinstance(mesh_or_scene, trimesh.Scene):
        all_meshes = get_scene_geometry(mesh_or_scene)
    else:
        all_meshes = [mesh_or_scene]

    print("Analyzing geometry and colors (Robust Mode)...")
    
    render_queue = []
    all_vertices = []
    
    processed_count = 0
    skipped_count = 0

    for mesh in all_meshes:
        # Sadece Mesh objelerini işle (Line/Point cloud gelirse atla)
        if not isinstance(mesh, trimesh.Trimesh):
            continue

        # --- KRİTİK ADIM: Geometri Temizliği ---
        # Bozuk indexleri temizlemeye çalış
        try:
            mesh.process(validate=False) # Hızlı temizlik
        except:
            pass # Temizlik başarısızsa olduğu gibi dene

        vertices = mesh.vertices
        faces = mesh.faces
        
        # Eğer yüzey veya nokta yoksa atla
        if len(faces) == 0 or len(vertices) == 0:
            continue
            
        # Global sınır hesabı için noktaları sakla
        all_vertices.append(vertices)

        # --- RENK ÇIKARMA (TRY-EXCEPT İLE KORUNMUŞ) ---
        face_colors = None
        try:
            # Önce görsel özellikleri renge çevirmeyi dene
            if hasattr(mesh.visual, 'to_color'):
                mesh.visual = mesh.visual.to_color()
            
            if hasattr(mesh.visual, 'face_colors'):
                 # BURASI PATLIYORDU, KORUMAYA ALDIK
                face_colors = mesh.visual.face_colors
            else:
                face_colors = np.tile(DEFAULT_OBSTACLE_COLOR, (len(faces), 1))
        
        except (IndexError, ValueError, AttributeError) as e:
            # Renk okuma hatası olursa, varsayılan engel rengini ata
            print(f"⚠️  Color warning on a mesh part: {e}. Using default grey.")
            face_colors = np.tile(DEFAULT_OBSTACLE_COLOR, (len(faces), 1))

        # Render kuyruğuna ekle
        for i, face in enumerate(faces):
            try:
                # İndeks kontrolü (Crash önleyici)
                if np.max(face) >= len(vertices):
                    continue

                points_3d = vertices[face]
                avg_z = np.mean(points_3d[:, 2])
                
                if avg_z < -0.5: # Çok derinleri çizme
                    continue
                
                # Renk varsa al, yoksa gri yap
                if face_colors is not None and i < len(face_colors):
                    color = tuple(face_colors[i])
                else:
                    color = DEFAULT_OBSTACLE_COLOR

                render_queue.append((avg_z, points_3d, color))
                processed_count += 1
                
            except Exception:
                skipped_count += 1
                continue

    if not all_vertices:
        print("❌ HATA: İşlenecek geçerli geometri bulunamadı.")
        return

    print(f"Geometry processed. Polygons: {processed_count}, Skipped/Fixed: {skipped_count}")

    # Sınırları hesapla
    all_vertices_concat = np.vstack(all_vertices)
    min_x, min_y, _ = np.min(all_vertices_concat, axis=0)
    max_x, max_y, _ = np.max(all_vertices_concat, axis=0)
    
    width_m = max_x - min_x
    height_m = max_y - min_y
    
    padding_px = 50
    width_px = int(np.ceil(width_m / RESOLUTION)) + (padding_px * 2)
    height_px = int(np.ceil(height_m / RESOLUTION)) + (padding_px * 2)
    
    print(f"Canvas Size: {width_px}x{height_px} px ({width_m:.2f}m x {height_m:.2f}m)")
    
    # Painter's Algorithm (Sıralama)
    render_queue.sort(key=lambda x: x[0]) 

    # Çizim
    image = Image.new('RGBA', (width_px, height_px), BACKGROUND_COLOR)
    draw = ImageDraw.Draw(image)
    
    def to_pixel(x, y):
        px = int((x - min_x) / RESOLUTION) + padding_px
        py = height_px - (int((y - min_y) / RESOLUTION) + padding_px)
        return (px, py)

    print("Rendering...")
    for _, points_3d, color in render_queue:
        polygon_pixels = [to_pixel(p[0], p[1]) for p in points_3d]
        draw.polygon(polygon_pixels, fill=color, outline=color)

    # Dosyaları Kaydet
    png_filename = f"{output_name}.png"
    image.save(png_filename)
    print(f"✅ Generated Image: {png_filename}")
    
    # YAML Oluştur
    origin_x = min_x - (RESOLUTION * padding_px)
    origin_y = min_y - (RESOLUTION * padding_px)
    
    yaml_content = {
        'image': png_filename,
        'mode': 'scale', 
        'resolution': RESOLUTION,
        'origin': [float(origin_x), float(origin_y), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    yaml_filename = f"{output_name}.yaml"
    with open(yaml_filename, 'w') as f:
        yaml.dump(yaml_content, f, sort_keys=False)
    print(f"✅ Generated YAML: {yaml_filename}")

if __name__ == "__main__":
    generate_colored_map("parkur.dae", output_name="colored_map")
