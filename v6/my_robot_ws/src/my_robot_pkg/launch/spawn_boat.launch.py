#!/bin/bash

# World dosyası yolu
WORLD_FILE="/home/beequeen/Downloads/my_robot_ws/install/my_robot_description/share/my_robot_description/worlds/boat_world.sdf"

# Model dosyası yolu
MODEL_FILE="/home/beequeen/Downloads/my_robot_ws/install/my_robot_description/share/my_robot_description/models/boat/model.sdf"

# Gazebo'yu arka planda başlat
gz sim $WORLD_FILE -r &

# Gazebo'nun başlaması için bekle
echo "Gazebo başlatılıyor... 5 saniye bekleniyor..."
sleep 5

# Robotu spawn et
echo "Robot spawn ediliyor..."
gz service -s /world/boat_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req "sdf_filename: \"$MODEL_FILE\", name: \"boat\", pose: {position: {x: 0, y: 0, z: 0.15}}"

echo "Robot spawn edildi!"