#!/usr/bin/env python3
import sys
import termios
import tty
import subprocess
import time

# ===== TOPIC İSİMLERİ =====
LEFT  = "/boat/left/thrust"
RIGHT = "/boat/right/thrust"

# ===== THRUST DEĞERLERİ (TRİMLİ) =====
LEFT_THRUST = 5.0      # Sol motor
RIGHT_THRUST = 5.0     # Sağ motor (sola kayma için biraz güçlü)
TURN_THRUST = 3.0
BRAKE_FORCE = -3.0

def send_thrust(topic, value):
    """Tek bir motora komut gönder"""
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {value}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

def move_forward():
    """İleri git"""
    send_thrust(LEFT, LEFT_THRUST)
    send_thrust(RIGHT, RIGHT_THRUST)

def move_backward():
    """Geri git"""
    send_thrust(LEFT, -LEFT_THRUST)
    send_thrust(RIGHT, -RIGHT_THRUST)

def turn_left():
    """Sola dön"""
    send_thrust(LEFT, -TURN_THRUST)
    send_thrust(RIGHT, TURN_THRUST)

def turn_right():
    """Sağa dön"""
    send_thrust(LEFT, TURN_THRUST)
    send_thrust(RIGHT, -TURN_THRUST)

def stop():
    """Durdur"""
    send_thrust(LEFT, 0.0)
    send_thrust(RIGHT, 0.0)

def brake():
    """Frenle"""
    send_thrust(LEFT, BRAKE_FORCE)
    send_thrust(RIGHT, BRAKE_FORCE)
    time.sleep(0.1)
    stop()

def getkey():
    """Terminal'den tek karakter oku"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

print("""
╔════════════════════════════════════╗
║       GEMİ KONTROL PANELİ         ║
╚════════════════════════════════════╝

  W : İleri git
  S : Geri git
  A : Sola dön
  D : Sağa dön
  X : Acil dur
  SPACE : Fren
  + : Sağ trim artır (sola kayma azalır)
  - : Sol trim artır (sağa kayma azalır)
  Q : Çık

Hazır. Tuşlara basın...
""")

try:
    trim_offset = 0.0  # Dinamik trim
    
    while True:
        k = getkey().lower()
        
        if k == "w":
            print(f"⬆️  İLERİ (Trim: {trim_offset:.2f})")
            send_thrust(LEFT, LEFT_THRUST)
            send_thrust(RIGHT, RIGHT_THRUST + trim_offset)
            
        elif k == "s":
            print("⬇️  GERİ")
            move_backward()
            
        elif k == "a":
            print("⬅️  SOLA DÖN")
            turn_left()
            
        elif k == "d":
            print("➡️  SAĞA DÖN")
            turn_right()
            
        elif k == "x":
            print("🛑 DURDUR")
            stop()
            
        elif k == " ":
            print("🟡 FREN")
            brake()
            
        elif k == "+":
            trim_offset += 0.1
            print(f"⚙️  Sağ trim: {trim_offset:.2f}")
            
        elif k == "-":
            trim_offset -= 0.1
            print(f"⚙️  Sağ trim: {trim_offset:.2f}")
            
        elif k == "q":
            print("\n🛑 Çıkılıyor...")
            stop()
            break
            
        elif k == "\x03":
            print("\n🛑 Kesildi...")
            stop()
            break
            
except KeyboardInterrupt:
    print("\n🛑 Kesildi...")
    stop()

print("Program sonlandı.")