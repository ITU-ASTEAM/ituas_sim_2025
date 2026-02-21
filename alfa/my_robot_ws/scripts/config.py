# ~/Desktop/my_robot_ws/scripts/config.py

# --- AYARLAR ---
WORLD_NAME = "ocean_world"  
ROBOT_NAME = "boat"        

# --- OTOMATİK PATH OLUŞTURUCU ---
# Gazebo topic yapısı genelde şöyledir: /world/{WORLD}/model/{ROBOT}/...
BASE_TOPIC = f"/world/{WORLD_NAME}/model/{ROBOT_NAME}"

class Topics:
    # Simülasyon Pozisyonu (Ground Truth)
    POSE = f"{BASE_TOPIC}/pose"
    
    # Sensörler (Link isimlerine dikkat et, URDF/SDF dosyanla aynı olmalı)
    # Eğer sensörler direkt model altındaysa 'link/base_link' kısmını silebilirsin.
    LIDAR = f"{BASE_TOPIC}/link/lidar_link/sensor/lidar/scan"
    CAMERA = f"{BASE_TOPIC}/link/camera_link/sensor/camera/image_raw"
    IMU    = f"{BASE_TOPIC}/link/base_link/sensor/imu/imu"
    
    # ROS2 Bridge üzerinden haberleşiyorsan topicler kısalabilir, kontrol et:
    CMD_VEL = f"/model/{ROBOT_NAME}/cmd_vel"

class Params:
    LINEAR_SPEED = 5.0
    ANGULAR_SPEED = 1.0
    TARGET_DISTANCE = 2.0  # Durma mesafesi vb.
