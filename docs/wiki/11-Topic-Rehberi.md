# 📡 Topic Rehberi

## Gazebo Transport Topic'leri

Bu projede ROS 2 bridge kullanılmadan doğrudan Gazebo Transport (`gz.transport13`) ile iletişim sağlanır.

---

## Topic Listesi

### Sensör Topic'leri

| Topic | Mesaj Tipi (Protobuf) | Python Import | Yayınlayan | Kullanan Script |
|---|---|---|---|---|
| `/lidar` | `LaserScan` | `gz.msgs10.laserscan_pb2` | GPU LiDAR sensör | `lidar_modulu.py`, `lidar.py` |
| `/camera/image_raw` | `Image` | `gz.msgs10.image_pb2` | RGB kamera | `kamera_modulu.py` |
| `/camera/depth/image_raw` | `Image` | `gz.msgs10.image_pb2` | Derinlik kamerası | `kamera_modulu.py` |

### Pozisyon Topic'leri

| Topic | Mesaj Tipi | Python Import | Yayınlayan | Kullanan Script |
|---|---|---|---|---|
| `/model/boat/pose` | `Pose_V` | `gz.msgs10.pose_v_pb2` | PosePublisher plugin | `position.py`, `boat_navigator.py` |
| `/world/ocean_world/model/boat/pose` | `Pose` | `gz.msgs10.pose_pb2` | Gazebo internal | `nav.py`, `lidar.py` |

> **⚠️ DİKKAT:** İki farklı pose topic'i var! `PosePublisher` plugin'i `/model/boat/pose` yayınlar (Pose_V tipi). Gazebo'nun kendi sistemi `/world/{world_name}/model/{model_name}/pose` yayınlar (Pose tipi). Scriptler farklı olanını kullanıyor.

### Motor Komut Topic'leri

| Topic | Mesaj Tipi | Python Import | Yön | Kullanan Script |
|---|---|---|---|---|
| `/model/boat/joint/leftmotor_link_joint/cmd_thrust` | `Double` | `gz.msgs10.double_pb2` | Subscribe (Gazebo okur) | `kontrol.py`, `nav.py`, `boat_navigator.py` |
| `/model/boat/joint/rightmotor_link_joint/cmd_thrust` | `Double` | `gz.msgs10.double_pb2` | Subscribe (Gazebo okur) | `kontrol.py`, `nav.py`, `boat_navigator.py` |

### Eski/Alternatif Topic'ler

Bu topic'ler `untested/` klasöründeki eski scriptlerde kullanılır:

| Topic | Kullanan | Not |
|---|---|---|
| `/boat/left/thrust` | `boat_keyboard.py` | Eski topic formatı |
| `/boat/right/thrust` | `boat_keyboard.py` | Eski topic formatı |
| `/model/boat/joint/left_motor_joint/cmd_thrust` | `boat_mission.py` | Farklı joint ismi |
| `/model/boat/joint/right_motor_joint/cmd_thrust` | `boat_mission.py` | Farklı joint ismi |
| `/world/boat_world/dynamic_pose/info` | `boat_pid_goto.py` | `boat_world` dünyası için |

---

## Topic Kontrol Komutları

### Topic'leri Listele
```bash
gz topic -l
```

### Bir Topic'i Dinle (Echo)
```bash
# Sürekli dinle
gz topic -e -t /model/boat/pose

# Tek mesaj al (JSON çıktı)
gz topic -e -n 1 --json-output -t /model/boat/pose

# LiDAR verisi
gz topic -e -t /lidar
```

### Topic'e Mesaj Gönder
```bash
# Sol motora itki komutu
gz topic -t /model/boat/joint/leftmotor_link_joint/cmd_thrust \
  -m gz.msgs.Double -p "data: -50.0"

# Sağ motora itki komutu
gz topic -t /model/boat/joint/rightmotor_link_joint/cmd_thrust \
  -m gz.msgs.Double -p "data: -50.0"
```

### Topic Bilgisi
```bash
gz topic -i -t /lidar
```

---

## Python'da Topic Kullanımı

### Subscribe (Veri Alma)

```python
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

node = Node()

def callback(msg):
    for p in msg.pose:
        print(f"x={p.position.x}, y={p.position.y}")

node.subscribe(Pose_V, "/model/boat/pose", callback)

# Program ayakta kalmalı (callback thread'de çalışır)
import time
while True:
    time.sleep(0.1)
```

### Publish (Veri Gönderme)

```python
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

node = Node()
pub = node.advertise("/model/boat/joint/leftmotor_link_joint/cmd_thrust", Double)

msg = Double()
msg.data = -80.0  # Negatif = ileri
pub.publish(msg)
```

---

## Mesaj Tipleri

### `Double`
```protobuf
message Double {
  double data = 1;
}
```

### `Pose_V` (Pose Vektörü)
```protobuf
message Pose_V {
  repeated Pose pose = 1;
}

message Pose {
  string name = 1;
  Vector3d position = 2;    // x, y, z
  Quaternion orientation = 3; // x, y, z, w
}
```

### `Image`
```protobuf
message Image {
  uint32 width = 1;
  uint32 height = 2;
  uint32 pixel_format_type = 3;
  bytes data = 4;  // RGB: uint8[], Depth: float32[]
}
```

### `LaserScan`
```protobuf
message LaserScan {
  double angle_min = 1;
  double angle_max = 2;
  double angle_step = 3;
  double range_min = 4;
  double range_max = 5;
  repeated double ranges = 6;
}
```

---

## ROS 2 Bridge (Opsiyonel)

Eğer ROS 2 araçları (rviz2, Nav2 vb.) kullanmak istersen bridge gerekir:

```bash
# LiDAR bridge
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan

# Veya bridge.yaml dosyasıyla
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=scripts/lidar/bridge.yaml
```

`scripts/lidar/bridge.yaml`:
```yaml
- topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "ignition.msgs.LaserScan"
  direction: GZ_TO_ROS
```
