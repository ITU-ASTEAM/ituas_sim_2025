# RViz Ayarları

Bu klasördeki `.rviz` dosyaları RViz2'nin hazır görünüm ayarlarıdır: hangi topic'lerin gösterileceğini, sabit çerçeveyi (Fixed Frame) ve araç çubuğunu içerirler. Hiçbir launch dosyası bunları otomatik yüklemez; elle açılırlar:

```bash
rviz2 -d tools/rviz/gazebov2.rviz

# Simülasyonda (Gazebo saati ile):
rviz2 -d tools/rviz/gazebov2.rviz --ros-args -p use_sim_time:=true
```

## Dosyalar

| Dosya | Fixed Frame | Açık ekranlar | Ne için |
|---|---|---|---|
| `gazebov2.rviz` | `map` | RobotModel (`/robot_description`), LaserScan `/scan`, Map `/map`, Path `/plan` ve `/local_plan` | Simülasyonda Nav2'yi izlemek: harita, lidar, global ve yerel plan. En kapsamlı ayar. |
| `forgazebosettings.rviz` | `map` | LaserScan `/scan`, Map `/map` | Simülasyonda harita ve lidar. `gazebov2.rviz`'in plan ekranları olmayan önceki hâli. |
| `forcamera.rviz` | `oak_rgb_camera_optical_frame` | PointCloud2 `/mistik_points`, LaserScan `/scan`, Map `/map` | OAK-D ile tespit edilen nesnelerin nokta bulutunu kamera çerçevesinde görmek. |
| `velocity.rviz` | `velodyne` | PointCloud2 `/velodyne` | Gerçek Velodyne VLP-16 lidarın ham nokta bulutunu sensör çerçevesinde görmek. Adı "velocity" olsa da Velodyne içindir. |

- `/mistik_points`: `my_robot_ws/src/objectdetect/objectdetect/pcl.py`'nin yayınladığı, YOLO ile tespit edilen nesnelerin derinlik kamerasından üretilmiş nokta bulutu.
- `gazebov2.rviz` dışındaki dosyalarda RobotModel ekranı var, ama description topic'i boş olduğu için robot modeli görünmez.

## Ortak araçlar

| Araç | Yayınladığı topic | Kullanım |
|---|---|---|
| 2D Pose Estimate | `/initialpose` | Başlangıç konumunu vermek |
| 2D Goal Pose | `/goal_pose` | Nav2'ye hedef göndermek |
| Publish Point | `/clicked_point` | Haritadan nokta seçmek |

## Gerçek donanım notları

- Velodyne bağlantısını kontrol etmek: `ping 192.168.1.201`. Ayar arayüzü `http://192.168.1.201/` adresindedir (https değil).
- OAK-D ROS sürücüsü (2025 ortamı, ROS 2 Humble): `sudo apt install ros-humble-depthai-ros`
