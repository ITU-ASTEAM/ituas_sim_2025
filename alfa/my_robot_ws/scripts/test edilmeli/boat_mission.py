#!/usr/bin/env python3
import subprocess
import time

LEFT  = "/model/boat/joint/left_motor_joint/cmd_thrust"
RIGHT = "/model/boat/joint/right_motor_joint/cmd_thrust"

THRUST = 10.0
TURN   = 8.0
SETTLE = 2.5

def pub(l, r):
    subprocess.run(
        ["gz", "topic", "-t", LEFT, "-m", "gz.msgs.Double", "-p", f"data: {l}"],
        stdout=subprocess.DEVNULL
    )
    subprocess.run(
        ["gz", "topic", "-t", RIGHT, "-m", "gz.msgs.Double", "-p", f"data: {r}"],
        stdout=subprocess.DEVNULL
    )

def stop_and_wait():
    pub(0, 0)
    time.sleep(SETTLE)

def go_forward(t):
    pub(THRUST, THRUST)
    time.sleep(t)
    stop_and_wait()

def go_backward(t):
    pub(-THRUST, -THRUST)
    time.sleep(t)
    stop_and_wait()

def turn_right(t):
    pub(-TURN, TURN)
    time.sleep(t)
    stop_and_wait()

def turn_left(t):
    pub(TURN, -TURN)
    time.sleep(t)
    stop_and_wait()

print("🚢 YENİ MISSION BAŞLADI")
time.sleep(2)

# 1️⃣ 50 metre ileri
go_forward(6.0)

# 2️⃣ Dur (zaten stop_and_wait içinde)

# 3️⃣ 90 derece sola dön
turn_left(0.8)

# 4️⃣ Dur

# 5️⃣ 20 metre ileri
go_forward(2.4)

# 6️⃣ 20 metre geri
go_backward(2.4)

# 7️⃣ Dur

# 8️⃣ 270 derece sağa dön
turn_right(2.4)

# 9️⃣ Dur

# 🔟 50 metre ileri
go_forward(6.0)

print("✅ MISSION TAMAMLANDI")
