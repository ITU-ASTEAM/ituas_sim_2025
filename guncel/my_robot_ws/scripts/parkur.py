#!/usr/bin/env python3
"""
parkur.py — 3 Aşamalı Görev Orkestrasyonu
==========================================
Aşama 1: boat_navigator.py  → YARI OTONOM, sabit waypoint takibi (Gz Transport direkt, Nav2 yok)
Aşama 2: waypoint_generator → TAM OTONOM, duba algılayarak Nav2 ile geçiş
Aşama 3: kamikaze.py        → KAMİKAZE, YOLO ile hedef bul ve çarp
"""

import sys
import time
import subprocess
import threading
import math

# ============================================================
#  AYARLAR
# ============================================================

# Aşama 1 — boat_navigator.py yolu
BOAT_NAVIGATOR_PATH = '/home/gauss/Desktop/my_robot_ws/scripts/missions/yarıotonom/boat_navigator.py'

# Aşama 2 — waypoint_generator ROS2 paketi
WAYPOINT_PKG     = 'algo'
WAYPOINT_NODE    = 'waypoint_generator'

# Aşama 3 — kamikaze.py yolu
KAMIKAZE_PATH    = '/home/gauss/Desktop/my_robot_ws/scripts/missions/kamikaze/kamikaze.py'

# Kamikaze hedef rengi — kullanıcıdan alınacak
KAMIKAZE_HEDEF   = None  # main() içinde set edilir

# ============================================================
#  YARDIMCI FONKSİYONLAR
# ============================================================

def banner(text):
    print('\n' + '=' * 50)
    print(f'  {text}')
    print('=' * 50)

def countdown(n):
    for i in range(n, 0, -1):
        print(f'  {i}...')
        time.sleep(1)

def stream_output(proc, prefix):
    """Subprocess çıktısını prefix ile ekrana yaz."""
    for line in iter(proc.stdout.readline, b''):
        decoded = line.decode('utf-8', errors='replace').rstrip()
        if decoded:
            print(f'[{prefix}] {decoded}')

# ============================================================
#  AŞAMA 1: BOAT NAVIGATOR
# ============================================================

def asama1_calistir():
    banner('AŞAMA 1 — YARI OTONOM NOKTA GÖREVİ')
    print(f'  Script: {BOAT_NAVIGATOR_PATH}')
    print()

    import os
    env = os.environ.copy()  # Mevcut tüm env variable'ları geçir (Gz dahil)

    proc = subprocess.Popen(
        [sys.executable, BOAT_NAVIGATOR_PATH],
        stdout=None,   # Direkt terminale yaz (pipe yok)
        stderr=None,
        env=env
    )
    proc.wait()

    if proc.returncode == 0:
        print('\n✅ AŞAMA 1 TAMAMLANDI!')
    else:
        print(f'\n⚠️  AŞAMA 1 bitti (return code: {proc.returncode})')

# ============================================================
#  AŞAMA 2: WAYPOINT GENERATOR (ROS2)
# ============================================================

def asama2_calistir(hedef_x, hedef_y):
    banner('AŞAMA 2 — TAM OTONOM NAVİGASYON (DUBA ALGILAMA)')
    print(f'  Hedef: ({hedef_x}, {hedef_y})')
    print()

    # waypoint_generator node'u başlat
    # Hedefi stdin üzerinden gönder
    cmd = ['ros2', 'run', WAYPOINT_PKG, WAYPOINT_NODE]

    proc = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1
    )

    # Çıktıyı canlı yaz
    t = threading.Thread(target=stream_output, args=(proc, 'WAYPOINT'), daemon=True)
    t.start()

    # Hedef koordinatı gönder (waypoint_generator input() bekliyor)
    time.sleep(3.0)  # Node'un başlaması için bekle
    goal_input = f'{hedef_x} {hedef_y}\n'.encode()
    try:
        proc.stdin.write(goal_input)
        proc.stdin.flush()
    except Exception as e:
        print(f'[UYARI] stdin yazma hatası: {e}')

    proc.wait()
    t.join(timeout=2.0)

    if proc.returncode == 0:
        print('\n✅ AŞAMA 2 TAMAMLANDI!')
    else:
        print(f'\n⚠️  AŞAMA 2 bitti (return code: {proc.returncode})')

# ============================================================
#  AŞAMA 3: KAMİKAZE
# ============================================================

def asama3_calistir(hedef_renk):
    banner('AŞAMA 3 — KAMİKAZE (YOLO HEDEF TAKİBİ)')
    print(f'  Hedef: {hedef_renk.upper()}')
    print()

    # kamikaze.py hedef rengini stdin'den alıyor
    import os
    kamikaze_dir = os.path.dirname(KAMIKAZE_PATH)
    proc = subprocess.Popen(
        [sys.executable, KAMIKAZE_PATH],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        cwd=kamikaze_dir  # kamera_modulu.py import için doğru dizin
    )

    # Çıktıyı canlı yaz
    t = threading.Thread(target=stream_output, args=(proc, 'KAMİKAZE'), daemon=True)
    t.start()

    # Hedef rengini gönder
    time.sleep(2.0)
    try:
        proc.stdin.write(f'{hedef_renk}\n'.encode())
        proc.stdin.flush()
    except Exception as e:
        print(f'[UYARI] stdin yazma hatası: {e}')

    proc.wait()
    t.join(timeout=2.0)

    print('\n🏁 AŞAMA 3 TAMAMLANDI — GÖREV BİTTİ!')

# ============================================================
#  KULLANICI GİRİŞİ
# ============================================================

def kullanici_girisi():
    print('\n' + '=' * 50)
    print('  🚢 PARKUR KONTROL SİSTEMİ')
    print('=' * 50)

    # Aşama 2 bitiş noktası — boat_navigator WAYPOINTS son elemanından otomatik
    # Değişirse buradan güncelle
    hedef_x, hedef_y = 83.9408, 15.4265  # box_14
    print(f'\n[AŞAMA 2] Bitiş noktası otomatik: ({hedef_x}, {hedef_y})')

    # Aşama 3 hedefi
    print('\n[AŞAMA 3] Kamikaze hedef rengi:')
    hedef_renk = input('  Renk (örn: siyah, yesil, kirmizi): ').strip()

    return hedef_x, hedef_y, hedef_renk

# ============================================================
#  ANA PROGRAM
# ============================================================

def main():
    # Tüm girişleri başta al
    hedef_x, hedef_y, hedef_renk = kullanici_girisi()

    print('\n\n📋 GÖREV ÖZETİ:')
    print(f'  Aşama 1: Yarı otonom nokta görevi (boat_navigator)')
    print(f'  Aşama 2: Tam otonom duba geçişi → ({hedef_x}, {hedef_y})')
    print(f'  Aşama 3: Kamikaze → [{hedef_renk.upper()}] hedefi')
    print()
    print('Başlamak için Enter\'a bas (Ctrl+C = iptal)...')
    input()

    start_time = time.time()

    try:
        # ── AŞAMA 1 ──────────────────────────────────────────
        asama1_calistir()

        print('\n⏳ Aşama 2 başlamadan önce bekleniyor...')
        countdown(3)

        # ── AŞAMA 2 ──────────────────────────────────────────
        asama2_calistir(hedef_x, hedef_y)

        print('\n⏳ Aşama 3 başlamadan önce bekleniyor...')
        countdown(3)

        # ── AŞAMA 3 ──────────────────────────────────────────
        asama3_calistir(hedef_renk)

    except KeyboardInterrupt:
        print('\n\n🛑 ACİL DURDURMA! Görev iptal edildi.')
        # Motorları durdur
        try:
            from gz.transport13 import Node
            from gz.msgs10.double_pb2 import Double
            _node = Node()
            _pl = _node.advertise('/model/boat/joint/leftmotor_link_joint/cmd_thrust', Double)
            _pr = _node.advertise('/model/boat/joint/rightmotor_link_joint/cmd_thrust', Double)
            _m = Double()
            _m.data = 0.0
            _pl.publish(_m)
            _pr.publish(_m)
            print('  Motorlar durduruldu.')
        except Exception:
            pass
        sys.exit(1)

    elapsed = time.time() - start_time
    banner(f'🏆 TÜM GÖREV TAMAMLANDI! ({elapsed:.0f}s)')


if __name__ == '__main__':
    main()
