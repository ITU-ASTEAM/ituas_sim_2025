#!/usr/bin/env python3
"""
FIXED NAVİGATOR - TEK PULSE STRATEJİSİ
Her iterasyonda sadece 1 pulse - momentum kontrolü
"""

import subprocess
import time
import math
import json

# ============= YAPILANDIRMA =============
POSE_TOPIC = "/world/boat_world/dynamic_pose/info"
LEFT  = "/model/boat/joint/left_motor_joint/cmd_thrust"
RIGHT = "/model/boat/joint/right_motor_joint/cmd_thrust"
MODEL = "boat"

# Waypoints (x, y, z)
WAYPOINTS = [
    (-55.745361328125, 4.2403202056884766, 0.5),
    (-42.77413739761049, 14.584090605933689, 0.5),
    (-29.985186429821233, 11.624106446693501, 0.5),
    (-24.751530660813394, 19.293566131667873, 0.5),
    (-14.272899627685547, 16.68000030517578, 0.5),
    (-2.8766901361853456, 16.039279094766393, 0.5),
    (10.079717096512439, 17.507263898661492, 0.5),
    (24.556473187474452, 19.051955908148695, 0.5),
    (31.733249948437532, 14.422172100623838, 0.5),
    (39.38311377480701, 11.812734749712423, 0.5)
]

# ÇOK DÜŞÜK HIZLAR - ZAMAN ÖNEMLI DEĞİL
TURN_THRUST = 0.8       # Daha yumuşak dönüş
FORWARD_THRUST = 1.0    # Daha yavaş ileri

# Pulse süreleri - ÇOK KISA!
PULSE_DURATION = 0.05   # Çok kısa pulse
PAUSE_DURATION = 0.4    # Daha uzun bekleme - tam durana kadar

# Toleranslar - DAHA DAR
POS_TOLERANCE = 2.5     # 2.5m - daha hassas
HEADING_TOLERANCE = 0.25  # ~14 derece - daha hassas

DT = 0.25  # Kontrol frekansı

# ============= YARDIMCI FONKSİYONLAR =============

def pub(left, right):
    """İki thruster'a itki gönderir"""
    subprocess.run(
        ["gz", "topic", "-t", LEFT, "-m", "gz.msgs.Double", "-p", f"data:{left}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    subprocess.run(
        ["gz", "topic", "-t", RIGHT, "-m", "gz.msgs.Double", "-p", f"data:{right}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def single_pulse(left, right):
    """TEK pulse - kısa ve kontrollü"""
    pub(left, right)
    time.sleep(PULSE_DURATION)
    pub(0, 0)
    time.sleep(PAUSE_DURATION)

def get_boat_pose():
    """Geminin güncel pozisyonunu ve yönünü okur"""
    try:
        raw = subprocess.check_output(
            ["gz", "topic", "-e", "-n", "1", "--json-output", "-t", POSE_TOPIC],
            timeout=2.0
        ).decode()
        
        raw = raw.strip().split("\n")[0]
        msg = json.loads(raw)
        
        for p in msg.get("pose", []):
            if p.get("name") == MODEL:
                pos = p.get("position", {})
                ori = p.get("orientation", {})
                
                if "x" in pos and "y" in pos:
                    x = pos["x"]
                    y = pos["y"]
                    
                    qw = ori.get("w", 1.0)
                    qx = ori.get("x", 0.0)
                    qy = ori.get("y", 0.0)
                    qz = ori.get("z", 0.0)
                    
                    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                                     1.0 - 2.0 * (qy * qy + qz * qz))
                    
                    return x, y, yaw
        
        return None
    
    except (subprocess.TimeoutExpired, json.JSONDecodeError, KeyError):
        return None

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def calculate_heading_error(x, y, yaw, target_x, target_y):
    desired_yaw = math.atan2(target_y - y, target_x - x)
    error = normalize_angle(desired_yaw - yaw)
    return error

# ============= NAVİGASYON =============

def navigate_to_waypoint(target_x, target_y, waypoint_num, total_waypoints):
    """Tek pulse stratejisi ile navigasyon"""
    print(f"\n{'='*60}")
    print(f"🎯 Waypoint {waypoint_num}/{total_waypoints}")
    print(f"   Hedef: X={target_x:.2f}m, Y={target_y:.2f}m")
    print(f"{'='*60}")
    
    STATE = "ROTATE"  # ROTATE → FORWARD → ADJUST
    start_time = time.time()
    iteration = 0
    
    pose = get_boat_pose()
    if not pose:
        print("❌ Pozisyon okunamadı!")
        return False
    
    start_x, start_y, _ = pose
    start_distance = calculate_distance(start_x, start_y, target_x, target_y)
    print(f"📍 Başlangıç: X={start_x:.2f}m, Y={start_y:.2f}m")
    print(f"📏 Mesafe: {start_distance:.2f}m\n")
    
    while True:
        iteration += 1
        loop_start = time.time()
        
        pose = get_boat_pose()
        if pose is None:
            print("⚠️  Pozissyon okunamadı")
            time.sleep(DT)
            continue
        
        x, y, yaw = pose
        distance = calculate_distance(x, y, target_x, target_y)
        heading_error = calculate_heading_error(x, y, yaw, target_x, target_y)
        
        # ============= DURUM MAKİNESİ =============
        
        if STATE == "ROTATE":
            if abs(heading_error) < HEADING_TOLERANCE:
                print(f"✅ Yön OK! İleri gidiliyor...")
                STATE = "FORWARD"
                time.sleep(0.5)
            else:
                # TEK PULSE ile dön
                if heading_error > 0:
                    single_pulse(-TURN_THRUST, TURN_THRUST)
                    direction = "⬅️"
                else:
                    single_pulse(TURN_THRUST, -TURN_THRUST)
                    direction = "➡️"
                
                if iteration % 2 == 0:
                    print(f"🔄 {direction} | Hata: {math.degrees(heading_error):5.1f}°")
        
        elif STATE == "FORWARD":
            # ULAŞTIK MI?
            if distance < POS_TOLERANCE:
                print(f"✅ Waypoint {waypoint_num} bölgesine girildi!")
                print(f"🔄 Pozisyon ve yön düzeltiliyor...")
                STATE = "ADJUST"
                pub(0, 0)
                time.sleep(1.0)  # Tam dur
                continue
            
            # YÖN KONTROLÜ - Daha hassas
            if abs(heading_error) > 0.4:  # ~23 derece
                print(f"⚠️  Yön düzeltme gerekli")
                STATE = "ROTATE"
                pub(0, 0)
                time.sleep(0.8)  # Daha uzun bekleme
                continue
            
            # MESAFEYE GÖRE ÇOOOK YAVAŞ THRUST
            if distance > 10.0:
                thrust = FORWARD_THRUST * 0.7  # %30 daha yavaş
            elif distance > 6.0:
                thrust = FORWARD_THRUST * 0.5  # %50 yavaş
            elif distance > 4.0:
                thrust = FORWARD_THRUST * 0.35 # %65 yavaş
            else:
                thrust = FORWARD_THRUST * 0.25 # %75 yavaş - sürünerek yaklaş
            
            # Minimal yön düzeltmesi
            if abs(heading_error) > 0.15:
                correction = heading_error * 0.15  # Daha yumuşak
                single_pulse(thrust - correction, thrust + correction)
            else:
                single_pulse(thrust, thrust)
            
            if iteration % 2 == 0:
                print(f"⛵ İLERİ | Mesafe: {distance:5.2f}m | Yaw: {math.degrees(heading_error):5.1f}°")
        
        elif STATE == "ADJUST":
            # WAYPOINT'TE DURUP YÖN VE POZİSYON DÜZELT
            
            # Yön yeterince iyi mi?
            if abs(heading_error) < 0.15:  # ~8 derece - çok hassas
                print(f"✅ Yön mükemmel!")
                
                # Pozisyon da iyi mi?
                if distance < 1.5:  # Çok yakın
                    print(f"🎉 Waypoint {waypoint_num} TAMAMLANDI!")
                    pub(0, 0)
                    time.sleep(2.0)  # Uzun dinlenme
                    return True
                else:
                    # Hafif yaklaş
                    print(f"📍 Pozisyon düzeltme: {distance:.2f}m kaldı")
                    single_pulse(FORWARD_THRUST * 0.2, FORWARD_THRUST * 0.2)
            else:
                # Yönü düzelt - çok yavaş
                print(f"🔄 Yön düzeltme: {math.degrees(heading_error):.1f}°")
                if heading_error > 0:
                    single_pulse(-TURN_THRUST * 0.5, TURN_THRUST * 0.5)
                else:
                    single_pulse(TURN_THRUST * 0.5, -TURN_THRUST * 0.5)
            
            if iteration % 2 == 0:
                print(f"🎯 DÜZELTME | Mesafe: {distance:.2f}m | Yaw: {math.degrees(heading_error):.1f}°")
        
        # Timeout
        if time.time() - start_time > 120.0:
            print(f"\n⚠️  Zaman aşımı!")
            pub(0, 0)
            return False
        
        # Döngü zamanlaması
        elapsed = time.time() - loop_start
        sleep_time = max(0, DT - elapsed)
        time.sleep(sleep_time)

# ============= ANA PROGRAM =============

def main():
    print("\n" + "="*60)
    print("🚢 TEK PULSE NAVİGASYON")
    print("="*60)
    print(f"📍 Toplam waypoint: {len(WAYPOINTS)}")
    print(f"⚙️  Strateji: Her iterasyon 1 pulse")
    print(f"🐌 Kontrollü momentum")
    print("="*60)
    
    print("\n⏳ Pozisyon okuma bekleniyor...")
    for _ in range(10):
        pose = get_boat_pose()
        if pose is not None:
            x, y, _ = pose
            print(f"✅ Başlangıç: X={x:.2f}m, Y={y:.2f}m\n")
            break
        time.sleep(0.5)
    else:
        print("❌ Pozisyon okunamadı!")
        return
    
    total = len(WAYPOINTS)
    completed = 0
    
    try:
        for i, waypoint in enumerate(WAYPOINTS, 1):
            target_x, target_y, _ = waypoint
            
            success = navigate_to_waypoint(target_x, target_y, i, total)
            
            if success:
                completed += 1
            else:
                print(f"⚠️  Waypoint {i} atlandı")
                time.sleep(1)
        
        print("\n" + "="*60)
        print("🎉 NAVİGASYON TAMAMLANDI!")
        print("="*60)
        print(f"✅ Tamamlanan: {completed}/{total}")
        print(f"📊 Başarı: {(completed/total)*100:.1f}%")
        print("="*60 + "\n")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Durduruldu!")
    
    finally:
        pub(0, 0)
        print("🛑 Motorlar durduruldu\n")

if __name__ == "__main__":
    main()