#!/usr/bin/env python3
"""
OTONOM GEMİ NAVİGASYON SİSTEMİ
Hedef: X=50m, Z=0m noktasına hassas duruş
- Pozisyon feedback ile kapalı döngü kontrol
- PID mantığı ile hedefe yaklaşırken yavaşlama
- Hedefi geçtiğinde aktif frenleme
- Savrulma kompanzasyonu
"""

import subprocess
import time
import math
import json
import sys

# ============= YAPILANDIRMA =============
POSE_TOPIC = "/world/boat_world/dynamic_pose/info"
LEFT  = "/model/boat/joint/left_motor_joint/cmd_thrust"
RIGHT = "/model/boat/joint/right_motor_joint/cmd_thrust"
MODEL = "boat"

# Kontrol parametreleri
MAX_THRUST = 5.0      # Maksimum itki
MIN_THRUST = 1       # Su sürtünmesini yenmek için minimum
KP_POS = 1.2           # Pozisyon P kazancı (mesafe → itki)
KI_POS = 0.05          # I kazancı (kümülatif hata)
KD_POS = 2.5           # D kazancı (hız değişimi)
KP_TURN = 5.0          # Yön düzeltme kazancı

# Hedef koordinatlar
TARGET_X = 30.0
TARGET_Z = 0.0

# Toleranslar
POS_EPS = 0.1          # ±10 cm kabul edilebilir
HEADING_EPS = 0.05     # ±3° yön hatası

# Zaman
DT = 0.05              # 50ms kontrol döngüsü (20Hz)
TIMEOUT = 120.0        # 2 dakika timeout

# PID hafızası
integral_error = 0.0
last_error = 0.0
last_velocity = 0.0

# ============= YARDIMCI FONKSİYONLAR =============

def clamp(value, min_val, max_val):
    """Değeri sınırlar içinde tutar"""
    return max(min_val, min(max_val, value))

def pub(left, right):
    """İki thruster'a itki gönderir"""
    subprocess.run(
        ["gz", "topic", "-t", LEFT, "-m", "gz.msgs.Double", "-p", f"data:{left}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    subprocess.run(
        ["gz", "topic", "-t", RIGHT, "-m", "gz.msgs.Double", "-p", f"data:{right}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

def get_boat_pose():
    """Geminin güncel pozisyonunu ve yönünü okur"""
    try:
        raw = subprocess.check_output(
            ["gz", "topic", "-e", "-n", "1", "--json-output", "-t", POSE_TOPIC],
            timeout=2.0
        ).decode()
        
        # İlk satırı al (JSON)
        raw = raw.strip().split("\n")[0]
        msg = json.loads(raw)
        
        # Boat modelini bul
        for p in msg.get("pose", []):
            if p.get("name") == MODEL:
                pos = p.get("position", {})
                ori = p.get("orientation", {})
                
                if "x" in pos and "z" in pos:
                    x = pos["x"]
                    z = pos["z"]
                    
                    # Quaternion → Yaw (Z ekseni rotasyonu)
                    qw = ori.get("w", 1.0)
                    qx = ori.get("x", 0.0)
                    qy = ori.get("y", 0.0)
                    qz = ori.get("z", 0.0)
                    
                    # Yaw hesapla (radyan)
                    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                                     1.0 - 2.0 * (qy * qy + qz * qz))
                    
                    return x, z, yaw
        
        return None
    
    except (subprocess.TimeoutExpired, json.JSONDecodeError, KeyError):
        return None

def calculate_distance(x, z, target_x, target_z):
    """Hedefe olan mesafeyi hesaplar"""
    return math.sqrt((target_x - x)**2 + (target_z - z)**2)

def calculate_heading_error(x, z, yaw, target_x, target_z):
    """Hedefe doğru bakması gereken açı ile mevcut açı arasındaki fark"""
    desired_yaw = math.atan2(target_z - z, target_x - x)
    error = desired_yaw - yaw
    
    # -π ile +π arasına normalize et
    while error > math.pi:
        error -= 2 * math.pi
    while error < -math.pi:
        error += 2 * math.pi
    
    return error

def active_brake(duration=0.2, brake_power=-15.0):
    """Aktif frenleme - karşı kuvvet uygular"""
    pub(brake_power, brake_power)
    time.sleep(duration)
    pub(0, 0)
    time.sleep(0.1)

# ============= ANA KONTROL DÖNGÜSÜ =============

def main():
    global integral_error, last_error, last_velocity
    
    print("=" * 60)
    print("🚢 OTONOM GEMİ NAVİGASYON SİSTEMİ")
    print("=" * 60)
    print(f"📍 Hedef: X={TARGET_X}m, Z={TARGET_Z}m")
    print(f"📏 Tolerans: ±{POS_EPS*100}cm")
    print(f"⏱️  Kontrol frekansı: {1/DT:.0f}Hz")
    print("=" * 60)
    print("\n⏳ Pozisyon okuma bekleniyor...\n")
    
    # İlk pozisyonu oku
    start_time = time.time()
    while True:
        pose = get_boat_pose()
        if pose is not None:
            start_x, start_z, _ = pose
            print(f"✅ Başlangıç pozisyonu: X={start_x:.2f}m, Z={start_z:.2f}m")
            break
        
        if time.time() - start_time > 10.0:
            print("❌ HATA: Gemi pozisyonu okunamadı!")
            return
        
        time.sleep(0.5)
    
    print(f"🎯 Hedef mesafe: {calculate_distance(start_x, start_z, TARGET_X, TARGET_Z):.2f}m")
    print("\n🚀 NAVİGASYON BAŞLADI!\n")
    
    mission_start = time.time()
    iteration = 0
    reached_target = False
    stable_count = 0  # Hedefe ulaşıldığında stabilite sayacı
    
    try:
        while time.time() - mission_start < TIMEOUT:
            iteration += 1
            loop_start = time.time()
            
            # Pozisyon oku
            pose = get_boat_pose()
            if pose is None:
                print("⚠️  Pozisyon okunamadı, tekrar deneniyor...")
                time.sleep(DT)
                continue
            
            x, z, yaw = pose
            
            # Hedefe mesafe ve yön hatası
            distance = calculate_distance(x, z, TARGET_X, TARGET_Z)
            heading_error = calculate_heading_error(x, z, yaw, TARGET_X, TARGET_Z)
            
            # Durumda mıyız?
            if distance < POS_EPS and abs(heading_error) < HEADING_EPS:
                stable_count += 1
                
                if stable_count == 1:
                    print(f"\n🎉 HEDEFE ULAŞILDI! (t={time.time()-mission_start:.1f}s)")
                    print(f"   Son konum: X={x:.3f}m, Z={z:.3f}m")
                    print(f"   Hata: {distance*100:.1f}cm")
                    reached_target = True
                
                # 2 saniye stabil kalırsa bitir
                if stable_count > 40:  # 40 * 0.05s = 2s
                    print("\n✅ GÖREV TAMAMLANDI - STABİL DURUŞ SAĞLANDİ\n")
                    pub(0, 0)
                    break
                
                # Hafif pozisyon tutma
                pub(0.5, 0.5)
            
            else:
                stable_count = 0  # Stabilite bozuldu
                
                # PID hesaplamaları
                error = distance
                integral_error += error * DT
                integral_error = clamp(integral_error, -10, 10)  # Anti-windup
                
                derivative = (error - last_error) / DT
                
                # PID çıktısı (ileri/geri itki)
                thrust_base = (KP_POS * error + 
                             KI_POS * integral_error + 
                             KD_POS * derivative)
                
                # Hedefe yaklaşırken yavaşla
                if distance < 5.0:
                    thrust_base *= 0.5
                if distance < 2.0:
                    thrust_base *= 0.3
                
                # Minimum itki kontrolü
                if abs(thrust_base) > 0 and abs(thrust_base) < MIN_THRUST:
                    thrust_base = MIN_THRUST if thrust_base > 0 else -MIN_THRUST
                
                # Hedefi geçtik mi? (X > TARGET_X)
                if x > TARGET_X + 0.3:
                    print(f"⚠️  HEDEFİ GEÇTİK! Aktif frenleme... (x={x:.2f}m)")
                    active_brake(duration=0.15, brake_power=-12.0)
                    integral_error = 0  # Reset PID
                    last_error = 0
                    continue
                
                # Yön düzeltmesi (diferansiyel itki)
                turn_correction = KP_TURN * heading_error
                
                # İtki komutları
                left_thrust = clamp(thrust_base - turn_correction, -MAX_THRUST, MAX_THRUST)
                right_thrust = clamp(thrust_base + turn_correction, -MAX_THRUST, MAX_THRUST)
                
                # Komutları gönder
                pub(left_thrust, right_thrust)
                
                # Hafıza güncelle
                last_error = error
                
                # Periyodik rapor (her 1 saniye)
                if iteration % 20 == 0:
                    print(f"📊 t={time.time()-mission_start:5.1f}s │ "
                          f"Mesafe: {distance:5.2f}m │ "
                          f"Pos: X={x:6.2f}m Z={z:5.2f}m │ "
                          f"Yaw: {math.degrees(yaw):5.1f}° │ "
                          f"Thrust: L={left_thrust:5.1f} R={right_thrust:5.1f}")
            
            # Döngü zamanlaması
            elapsed = time.time() - loop_start
            sleep_time = max(0, DT - elapsed)
            time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Kullanıcı tarafından durduruldu!")
    
    finally:
        # Temizlik
        pub(0, 0)
        
        if reached_target:
            final_pose = get_boat_pose()
            if final_pose:
                fx, fz, _ = final_pose
                final_error = calculate_distance(fx, fz, TARGET_X, TARGET_Z)
                print("\n" + "=" * 60)
                print("📊 GÖREV RAPORU")
                print("=" * 60)
                print(f"✅ Hedef: X={TARGET_X}m, Z={TARGET_Z}m")
                print(f"📍 Ulaşılan: X={fx:.3f}m, Z={fz:.3f}m")
                print(f"📏 Hata: {final_error*100:.2f}cm")
                print(f"⏱️  Süre: {time.time()-mission_start:.1f}s")
                print(f"🎯 Doğruluk: {(1 - final_error/distance)*100:.1f}%")
                print("=" * 60 + "\n")

if __name__ == "__main__":
    main()