# config.py

# --- DEĞİŞKENLER (Sadece burayı değiştireceksin) ---
WORLD_NAME = "ocean_world"  # Simülasyondaki dünya ismi
ROBOT_NAME = "boat"         # SDF/URDF dosyasındaki robot ismi
# ---------------------------------------------------

# Gazebo/Ignition Simülasyonu için Dinamik Prefix Oluşturucu
# Genelde yapı şöyledir: /world/{WORLD}/model/{ROBOT}/...
SIM_PREFIX = f"/world/{WORLD_NAME}/model/{ROBOT_NAME}"

# Link isimleri (URDF/SDF dosyanda tanımlı olanlar)
BASE_LINK = "base_link"
LIDAR_LINK = "lidar_link"
CAMERA_LINK = "camera_link"

class Topics:
    # 1. Navigasyon & Pozisyon
    POSE = f"{SIM_PREFIX}/pose"
    ODOMETRY = f"{SIM_PREFIX}/link/{BASE_LINK}/sensor/imu/imu" # Örnek IMU yolu
    
    # 2. Lidar (Genelde link ve sensor ismiyle uzar)
    # Örn: /world/ocean_world/model/boat/link/lidar_link/sensor/lidar/scan
    LIDAR_SCAN = f"{SIM_PREFIX}/link/{LIDAR_LINK}/sensor/lidar/scan"
    LIDAR_POINTS = f"{SIM_PREFIX}/link/{LIDAR_LINK}/sensor/lidar/points"

    # 3. Kamera
    CAM_IMAGE_RAW = f"{SIM_PREFIX}/link/{CAMERA_LINK}/sensor/camera/image"
    CAM_INFO = f"{SIM_PREFIX}/link/{CAMERA_LINK}/sensor/camera/camera_info"
    
    # 4. Kontrol (ROS2 Bridge ile genelde basitleştirilir ama ham hali şöyledir)
    CMD_VEL = f"/model/{ROBOT_NAME}/cmd_vel"  # Bazen sadece bu kadar olur
