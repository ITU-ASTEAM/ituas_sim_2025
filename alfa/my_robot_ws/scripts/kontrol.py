#!/usr/bin/env python3
import sys
import termios
import tty
import select
from gz.transport13 import Node 
from gz.msgs10.double_pb2 import Double

# --- AYARLAR ---
FORWARD_THRUST = 40.0   # İleri
BACK_THRUST = -40.0     # Geri
TURN_THRUST = 40.0      # Dönüş (Daha güçlü olsun diye artırdım)

LEFT_TOPIC = "/model/boat/joint/leftmotor_link_joint/cmd_thrust"
RIGHT_TOPIC = "/model/boat/joint/rightmotor_link_joint/cmd_thrust"

msg = """
---------------------------
GEMİ KONTROL PANELİ (DÜZELTİLMİŞ)
---------------------------
W: İLERİ (Sol:80, Sağ:80) -> Düz gitmeli
S: GERİ  (Sol:-60, Sağ:-60)
A: SOLA  (Sol:-80, Sağ:80) -> Olduğu yerde döner
D: SAĞA  (Sol:80, Sağ:-80) -> Olduğu yerde döner

BOŞLUK: Motorları Durdur

CTRL-C: Çıkış
---------------------------
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_thrust(node, pub_left, pub_right, left_val, right_val):
    # Mesajları oluştur
    msg_left = Double()
    msg_left.data = left_val
    
    msg_right = Double()
    msg_right.data = right_val
    
    # Yayınla
    pub_left.publish(msg_left)
    pub_right.publish(msg_right)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    node = Node()
    pub_left = node.advertise(LEFT_TOPIC, Double)
    pub_right = node.advertise(RIGHT_TOPIC, Double)

    print(msg)
    
    target_left = 0.0
    target_right = 0.0

    try:
        while(1):
            key = getKey()
            
            # --- DÜZELTİLMİŞ MANTIK ---
            if key == 'w':
                # İLERİ İÇİN İKİSİ DE POZİTİF OLMALI
                target_left = BACK_THRUST
                target_right = BACK_THRUST
             
                print(f"\rDurum: İLERİ TAM GAZ ({FORWARD_THRUST}N)      ", end="")
                
            elif key == 's':
                # GERİ İÇİN İKİSİ DE NEGATİF OLMALI
                target_left = FORWARD_THRUST
                target_right = FORWARD_THRUST 
                print(f"\rDurum: GERİ GELİYOR ({BACK_THRUST}N)          ", end="")
                
            elif key == 'a':
                # SOLA DÖNMEK İÇİN: SOL GERİ, SAĞ İLERİ
                target_left = TURN_THRUST
                target_right = -TURN_THRUST
                print(f"\rDurum: SOLA DÖNÜYOR                           ", end="")
                
            elif key == 'd':
                # SAĞA DÖNMEK İÇİN: SOL İLERİ, SAĞ GERİ
                target_left = -TURN_THRUST
                target_right = TURN_THRUST
                print(f"\rDurum: SAĞA DÖNÜYOR                           ", end="")
                
            elif key == ' ': 
                target_left = 0.0
                target_right = 0.0
                print(f"\rDurum: MOTORLAR DURDU (Süzülme)               ", end="")

            elif key == '\x03':
                break

            publish_thrust(node, pub_left, pub_right, target_left, target_right)

    except Exception as e:
        print(f"\nHata: {e}")

    finally:
        publish_thrust(node, pub_left, pub_right, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nProgram kapatıldı.")
