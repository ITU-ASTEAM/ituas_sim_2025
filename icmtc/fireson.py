#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
#  ITU AS Team - ENTEGRE OTONOM SISTEM
#  Waypoint (GPS) Navigasyonu + OAK-D Sekil Tespit & Gorsel Yaklasma
#
#  MODLAR (tek tusla gecis):
#    g : GUIDED  -> GPS waypoint takibi (setpoint_position/local)
#    m : MANUAL  -> Terminalden tusla motor surusu (toggle)
#    k : SEKIL   -> Kamera ile sekil tespit + otonom ortalama / 1 m yaklasma
#
#  SEKIL modu akisi:
#    'k'  -> hedef renk & sekil sorulur -> ArduPilot MANUAL'e alinir
#         -> sekil ONAY_KARE kadar ust uste gorulunce ONAYLANIR
#         -> gorsel servo baslar: hedefi yatayda ORTALAR ve ~1 m mesafeye
#            kadar YAKLASIR
#         -> ulasinca motorlari durdurup HOLD'a gecer. Yeni sekil icin tekrar 'k'.
#
#  NOT (tasarim): Kamerayi hedefe "ortalamak" ve tam 1 m'de durmak, GPS pozisyon
#  setpoint'i ile degil, goruntuden anlik direksiyon/gaz uretilerek (gorsel servo)
#  yapilir. Hareket yine senin waypoint kodundaki mavros/RC-override altyapisi
#  uzerinden gider. GUIDED-hiz (velocity) tabanli bir varyant da eklenebilir;
#  o pusula/heading + kamera FOV kalibrasyonu ister.
# =============================================================================

import cv2
import numpy as np
import depthai as dai
import math
import statistics
import threading
import time
import sys
import os
import json
import termios
import tty
import select
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool

# ============================== AYARLAR ======================================
# --- Ekran ---
SHOW_WINDOW = True          # SSH/bassiz calistirmada False yap (cv2 penceresi acilmaz)

# --- config.json: dosya yoksa/bozuksa asagidaki DEFAULTS ile calismaya devam edilir ---
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

DEFAULTS = {
    "kamera": {"preview_w": 1024, "preview_h": 768},
    "gps": {"kalibrasyon_n": 10, "ema_alpha": 0.12, "hedef_toleransi": 1.2, "varsayilan_hiz": 0.5},
    "rc": {"kanal_dumen": 0, "kanal_gaz": 2, "dumen_yonu": 1},
    "sekil_servo": {
        "hedef_mesafe_m": 1.0, "mesafe_tol_m": 0.15, "merkez_tol": 0.08,
        "kp_steer": 0.9, "kp_mesafe": 0.35, "max_steer": 0.55,
        "max_yaklasma_hizi": 0.45, "hiz_kesme": 1.0, "onay_kare": 5,
        "ulasildi_sure_s": 0.6, "depth_timeout_s": 0.4,
    },
    "pompa": {"otomatik_ates": True, "su_sikma_suresi_s": 2.5, "komut_ac": "F", "komut_kapat": "F"},
    "gosterge": {"komut_guided": "X", "komut_manual": "Y"},
    "arduino": {"port_secenekleri": ["/dev/ttyCH341USB0", "/dev/ttyACM0", "/dev/ttyACM1"], "baudrate": 115200, "baglanti_timeout_s": 1},
}


def load_config():
    """config.json'u okur; eksik bolum/anahtarlari DEFAULTS ile tamamlar."""
    cfg = {bolum: dict(deger) for bolum, deger in DEFAULTS.items()}
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            disk = json.load(f)
        for bolum, anahtarlar in DEFAULTS.items():
            disk_bolum = disk.get(bolum, {})
            if not isinstance(disk_bolum, dict):
                continue
            for anahtar in anahtarlar:
                if anahtar in disk_bolum:
                    cfg[bolum][anahtar] = disk_bolum[anahtar]
    except FileNotFoundError:
        print(f"[UYARI] {CONFIG_PATH} bulunamadi, varsayilan ayarlarla devam ediliyor.")
    except (json.JSONDecodeError, OSError) as e:
        print(f"[UYARI] config.json okunamadi ({e}), varsayilan ayarlarla devam ediliyor.")
    return cfg


CFG = load_config()

# --- Kamera / goruntu ---
PREVIEW_W, PREVIEW_H = CFG["kamera"]["preview_w"], CFG["kamera"]["preview_h"]

# --- GPS / Waypoint ---
KALIBRASYON_N = CFG["gps"]["kalibrasyon_n"]
EMA_ALPHA = CFG["gps"]["ema_alpha"]
HEDEF_TOLERANSI = CFG["gps"]["hedef_toleransi"]
VARSAYILAN_HIZ = CFG["gps"]["varsayilan_hiz"]

# --- RC kanallari (ArduPilot skid-steer karisimini KENDISI yapar) ---
KANAL_DUMEN = CFG["rc"]["kanal_dumen"]   # CH1 -> direksiyon girisi
KANAL_GAZ = CFG["rc"]["kanal_gaz"]       # CH3 -> gaz girisi
DUMEN_YONU = CFG["rc"]["dumen_yonu"]     # motorlar ters calisirsa -1 yap

# --- SEKIL GORSEL SERVO (yaklasma) parametreleri ---
HEDEF_MESAFE = CFG["sekil_servo"]["hedef_mesafe_m"]              # metre (hedef standoff)
MESAFE_TOL = CFG["sekil_servo"]["mesafe_tol_m"]                  # metre tolerans
MERKEZ_TOL = CFG["sekil_servo"]["merkez_tol"]                    # normalize yatay hata toleransi
KP_STEER = CFG["sekil_servo"]["kp_steer"]                        # direksiyon oransal kazanci
KP_MESAFE = CFG["sekil_servo"]["kp_mesafe"]                      # mesafe oransal kazanci (1/m)
MAX_STEER = CFG["sekil_servo"]["max_steer"]                      # maks direksiyon komutu (0..1)
MAX_YAKLASMA_HIZI = CFG["sekil_servo"]["max_yaklasma_hizi"]      # maks ileri gaz (0..1)
HIZ_KESME = CFG["sekil_servo"]["hiz_kesme"]                      # merkezde degilken ileri hizi kisma faktoru
ONAY_KARE = CFG["sekil_servo"]["onay_kare"]                      # kac kare ust uste gorulurse ONAY
ULASILDI_SURE = CFG["sekil_servo"]["ulasildi_sure_s"]            # kac saniye hedefte kalirsa "ulasildi"
DEPTH_TIMEOUT = CFG["sekil_servo"]["depth_timeout_s"]            # son gecerli derinlik bu sureden eskiyse gecersiz say

# --- POMPA (role) ---
POMPA_OTOMATIK_ATES = CFG["pompa"]["otomatik_ates"]
POMPA_SU_SIKMA_SURESI = CFG["pompa"]["su_sikma_suresi_s"]
POMPA_KOMUT_AC = CFG["pompa"]["komut_ac"]
POMPA_KOMUT_KAPAT = CFG["pompa"]["komut_kapat"]

# --- GOSTERGE LED (Arduino) ---
GOSTERGE_KOMUT_GUIDED = CFG["gosterge"]["komut_guided"]
GOSTERGE_KOMUT_MANUAL = CFG["gosterge"]["komut_manual"]

# --- ARDUINO seri baglanti ---
ARDUINO_PORT_SECENEKLERI = CFG["arduino"]["port_secenekleri"]
ARDUINO_BAUDRATE = CFG["arduino"]["baudrate"]
ARDUINO_TIMEOUT = CFG["arduino"]["baglanti_timeout_s"]

FONT = cv2.FONT_HERSHEY_SIMPLEX


# ======================= HASSAS SEKIL ANALIZI ================================
# ======================= HASSAS SEKIL ANALIZI ================================
def get_shape_name(cnt):
    area = cv2.contourArea(cnt)
    # Çok küçük konturları baştan ele
    if area < 800: # Firefighting kodundaki gibi 800 yaptık, daha stabil olur
        return None, None, 0, 0, 0, 0

    x, y, w, h = cv2.boundingRect(cnt)
    
    # --- Firefighting Kodu Köşe (Corner) ve Oran Analizi ---
    perimeter = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.04 * perimeter, True)
    corner_count = len(approx)

    aspect_ratio = float(w) / h if h > 0 else 0

    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity = float(area) / hull_area if hull_area > 0 else 0

    shape_result = None
    
    # Firefighting hedef (gemi/engel) tespit şartları: 
    has_right_corners = 5 <= corner_count <= 10
    is_horizontal = aspect_ratio > 1.2
    is_solid = solidity > 0.65

    # Eğer şartlar sağlanıyorsa bu bizim gemimizdir
    if has_right_corners and is_horizontal and is_solid:
        shape_result = "GEMI"

    # Dönüş değerleri: Ekrana (D: köşe sayısı, S: solidity) yazdırabilmesi için sırayı koruduk
    return shape_result, approx, corner_count, round(solidity, 2), 0, round(aspect_ratio, 2)


def depth_ortala(depth_frame, cX, cY, roi=4):
    """RGB preview koordinatini depth cozunurlugune olcekleyip ROI medyani (mm) dondurur.
       Depth (400p) ile preview (1024x768) cozunurlukleri farkli oldugu icin olcekleme sart."""
    h, w = depth_frame.shape[:2]
    dx = int(cX * w / PREVIEW_W)
    dy = int(cY * h / PREVIEW_H)
    if not (0 <= dy < h and 0 <= dx < w):
        return 0.0
    y1, y2 = max(0, dy - roi), min(h, dy + roi)
    x1, x2 = max(0, dx - roi), min(w, dx + roi)
    roi_d = depth_frame[y1:y2, x1:x2]
    gecerli = roi_d[roi_d > 0]
    if gecerli.size == 0:
        return 0.0
    return float(np.median(gecerli))


# ============================ ANA NODE ======================================
class EntegreNode(Node):
    def __init__(self):
        super().__init__('entegre_otonom_node')

        # --- Yayinci / Aboneler ---
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.detect_pub = self.create_publisher(String, '/detection/target_status', 10)

        # --- Servis istemcileri ---
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # --- 50 Hz RC dongusu ---
        self.timer = self.create_timer(0.02, self.publish_rc_logic)

        # --- Mod ---
        self.mod = 'GUIDED'          # 'GUIDED' | 'MANUAL' | 'SEKIL'
        self.kapat = False
        self.kamera_var = True

        # --- GPS durum ---
        self.is_calibrated = False
        self.reached_target = False
        self.ref_lat, self.ref_lon = None, None
        self.target_lat, self.target_lon = 0.0, 0.0
        self.hedef_n, self.hedef_e = 0.0, 0.0
        self.ema_n_v, self.ema_e_v = None, None
        self.kal_lat_buf, self.kal_lon_buf = [], []
        self.hedef_bekleniyor = False
        self.hiz = VARSAYILAN_HIZ

        # --- Manuel motor toggle durumlari (-1 geri, 0 dur, +1 ileri) ---
        self.sol_durum = 0
        self.sag_durum = 0

        # --- SEKIL modu durumlari ---
        self.target_color = "YOK"
        self.target_shape = "YOK"
        self.sekil_bekleniyor = False
        self.confirmation_count = 0
        self.required_frames = ONAY_KARE
        self.sekil_onaylandi = False
        self.sekil_gorunuyor = False
        self.sekil_tamamlandi = False
        self.sekil_reached_t0 = None
        self.sekil_cX = PREVIEW_W // 2
        self.sekil_cY = PREVIEW_H // 2
        self.sekil_dist = 0.0
        self.sekil_dist_time = 0.0
        self.sekil_last_print = 0.0
        self.sekil_lock = threading.Lock()

        # --- POMPA (role) durumlari ---
        self.pompa_atildi = False
        self.pompa_timer = None

        # --- Arduino (gosterge LED + pompa rolesi) ---
        self.arduino = None
        self.arduino_baglan()

        # --- Terminal (stdin) dinleyici thread'i ---
        self.klavye_thread = threading.Thread(target=self.klavye_dongusu, daemon=True)
        self.klavye_thread.start()

        self.get_logger().info(' SISTEM BASLATILDI: Waypoint + Sekil Tespit entegre.')
        self.yardim_yazdir()

    # ---------------------------------------------------------------------
    def arduino_baglan(self):
        """Kullanicidan seri port secmesini ister ve Arduino'ya baglanir.
        Baglanamazsa sistemi durdurmaz: gosterge LED ve pompa devre disi kalir,
        GPS/MANUAL/SEKIL kontrolu normal calismaya devam eder."""
        print("\n" + "=" * 62)
        print(" ARDUINO BAGLANTISI (gosterge LED + pompa rolesi)")
        secenekler = ARDUINO_PORT_SECENEKLERI
        for i, p in enumerate(secenekler, 1):
            print(f"   {i}) {p}")
        elle_secim = len(secenekler) + 1
        print(f"   {elle_secim}) Elle port gir")
        print(f"   0) Arduino baglama (LED/pompa devre disi)")
        try:
            secim = input(f" Port sec [0-{elle_secim}]: ").strip()
            if secim == "0" or secim == "":
                print(" Arduino baglantisi atlandi.")
                return
            if secim == str(elle_secim):
                port = input(" Port yolu gir (orn. /dev/ttyACM0): ").strip()
            else:
                port = secenekler[int(secim) - 1]
            self.arduino = serial.Serial(port, ARDUINO_BAUDRATE, timeout=ARDUINO_TIMEOUT)
            time.sleep(2.0)                      # CH340 DTR reset -> bootloader'i bekle
            self.arduino.reset_input_buffer()
            print(f" Arduino baglandi: {port}")
        except Exception as e:
            self.arduino = None
            print(f" [UYARI] Arduino baglanamadi ({e}). Gosterge LED ve pompa devre disi.")
        print("=" * 62 + "\n")

    def arduino_yaz(self, komut):
        if self.arduino is None:
            return
        try:
            self.arduino.write(komut.encode())
        except Exception as e:
            self.get_logger().error(f' Arduino yazma hatasi: {e}')

    # ---------------------------------------------------------------------
    def yardim_yazdir(self):
        print("\n" + "=" * 66)
        print(" ENTEGRE OTONOM SISTEM - KONTROLLER")
        print(" MODLAR")
        print("   g : GUIDED (GPS waypoint takibi)")
        print("   m : MANUAL (tusla motor surusu)")
        print("   k : SEKIL  (kamera ile tespit + otonom 1 m yaklasma)")
        print(" MANUEL MOTOR (toggle: bas=baslat, tekrar bas=durdur)")
        print("   7:SOL ILERI  9:SAG ILERI  1:SOL GERI  3:SAG GERI")
        print("   w:ikisi ileri  s:ikisi geri  a:sola pivot  d:saga pivot")
        print("   5 / SPACE : TUM MOTORLAR DUR")
        print(" DIGER")
        print("   +/- : hiz (%.1f)   t:ARM   y:DISARM   h:yeni GPS hedefi" % self.hiz)
        print("   q   : (kamera penceresi seciliyken) programi kapat")
        print(" POMPA MANUEL TEST (kamera/servo akisindan bagimsiz, dogrudan role testi)")
        print(f"   p : pompayi ates (ac -> {POMPA_SU_SIKMA_SURESI:.1f} sn sonra otomatik kapanir)")
        print("   o : pompayi manuel kapat / bayragi sifirla (tekrar 'p' icin)")
        print(" NOT: SEKIL/MANUEL modda motorlarin donmesi icin once ARM (t) gerekir.")
        print(" ARDUINO: g/m tuslari gosterge LED'ini de degistirir. SEKIL modunda hedefe")
        print(f"          ulasilinca pompa {POMPA_SU_SIKMA_SURESI:.1f} sn otomatik acilir/kapanir"
              f" (otomatik_ates={POMPA_OTOMATIK_ATES}).")
        print("=" * 66 + "\n")

    def calculate_pwm(self, axis_val):
        val = 1500 + int(axis_val * 400.0)
        if abs(val - 1500) < 30:
            return 1500
        return max(1100, min(1900, val))

    def durum_yaz(self):
        isim = {-1: 'GERI ', 0: 'DUR  ', 1: 'ILERI'}
        print(f"\n[MOTOR] SOL: {isim[self.sol_durum]} | SAG: {isim[self.sag_durum]} | Hiz: {self.hiz:.1f}")

    # ======================= TERMINAL DINLEYICI ==========================
    def klavye_dongusu(self):
        self.term_ayarlari = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok() and not self.kapat:
                # Kullanicidan satir girisi beklenen durumlar (terminal NORMAL modda)
                if self.hedef_bekleniyor:
                    self.kullanici_hedef_al()
                    continue
                if self.sekil_bekleniyor:
                    self.kullanici_sekil_al()
                    continue

                # Tek tus okuma (cbreek)
                tty.setcbreak(sys.stdin.fileno())
                okunabilir, _, _ = select.select([sys.stdin], [], [], 0.1)
                tus = sys.stdin.read(1) if okunabilir else ''
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_ayarlari)

                if tus:
                    self.tus_isle(tus.lower())
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_ayarlari)

    def tus_isle(self, t):
        # --- MANUEL motor toggle tuslari ---
        if t == '7':
            self.sol_durum = 1 if self.sol_durum != 1 else 0; self.durum_yaz()
        elif t == '1':
            self.sol_durum = -1 if self.sol_durum != -1 else 0; self.durum_yaz()
        elif t == '9':
            self.sag_durum = 1 if self.sag_durum != 1 else 0; self.durum_yaz()
        elif t == '3':
            self.sag_durum = -1 if self.sag_durum != -1 else 0; self.durum_yaz()
        elif t == 'w':
            hedef = 0 if (self.sol_durum, self.sag_durum) == (1, 1) else 1
            self.sol_durum = self.sag_durum = hedef; self.durum_yaz()
        elif t == 's':
            hedef = 0 if (self.sol_durum, self.sag_durum) == (-1, -1) else -1
            self.sol_durum = self.sag_durum = hedef; self.durum_yaz()
        elif t == 'a':
            if (self.sol_durum, self.sag_durum) == (-1, 1):
                self.sol_durum = self.sag_durum = 0
            else:
                self.sol_durum, self.sag_durum = -1, 1
            self.durum_yaz()
        elif t == 'd':
            if (self.sol_durum, self.sag_durum) == (1, -1):
                self.sol_durum = self.sag_durum = 0
            else:
                self.sol_durum, self.sag_durum = 1, -1
            self.durum_yaz()
        elif t in (' ', '5'):
            self.sol_durum = self.sag_durum = 0
            print("\n TUM MOTORLAR DUR (1500).")

        # --- Hiz ---
        elif t == '+' or t == '=':
            self.hiz = min(1.0, self.hiz + 0.1); print(f"\n Hiz: {self.hiz:.1f}")
        elif t == '-':
            self.hiz = max(0.1, self.hiz - 0.1); print(f"\n Hiz: {self.hiz:.1f}")

        # --- ARM / DISARM ---
        elif t == 't':
            req = CommandBool.Request(); req.value = True; self.arm_client.call_async(req)
            self.get_logger().info(' ARM Istegi Gonderildi.')
        elif t == 'y':
            self.sol_durum = self.sag_durum = 0
            req = CommandBool.Request(); req.value = False; self.arm_client.call_async(req)
            self.get_logger().warn(' DISARM Istegi Gonderildi.')

        # --- MODLAR ---
        elif t == 'm':
            self.mod = 'MANUAL'
            self.sol_durum = self.sag_durum = 0
            req = SetMode.Request(); req.custom_mode = 'MANUAL'; self.mode_client.call_async(req)
            self.arduino_yaz(GOSTERGE_KOMUT_MANUAL)
            self.get_logger().warn('\n MOD: MANUAL (TUS SURUSU)')
        elif t == 'g':
            self.mod = 'GUIDED'
            self.sol_durum = self.sag_durum = 0
            req = SetMode.Request(); req.custom_mode = 'GUIDED'; self.mode_client.call_async(req)
            self.reached_target = False
            self.arduino_yaz(GOSTERGE_KOMUT_GUIDED)
            self.get_logger().info('\n MOD: GUIDED (OTONOM GPS TAKIP)')
        elif t == 'k':
            if not self.kamera_var:
                self.get_logger().error(' Kamera bulunamadi, SEKIL modu kullanilamaz.')
            elif not self.sekil_bekleniyor:
                self.mod = 'SEKIL'
                self.sol_durum = self.sag_durum = 0
                self.sekil_bekleniyor = True
                self.get_logger().info('\n MOD: SEKIL TESPIT - hedef sekil isteniyor...')
        elif t == 'h':
            if not self.hedef_bekleniyor and self.is_calibrated:
                self.sol_durum = self.sag_durum = 0
                self.hedef_bekleniyor = True
                self.get_logger().info('\n Yeni GPS Hedefi isteniyor...')

        # --- POMPA MANUEL TEST (kamera/servo akisindan bagimsiz, roleattirma.py
        #     ile ayni yontemle: arduino.write(komut.encode())) ---
        elif t == 'p':
            self.get_logger().info('\n [POMPA TEST] Manuel ateslendi (roleattirma.py ile ayni yontem).')
            self.pompa_ates()
        elif t == 'o':
            if self.pompa_timer is not None:
                self.pompa_timer.cancel()
            self.pompa_atildi = False
            self.pompa_kapat()
            self.get_logger().info('\n [POMPA TEST] Manuel kapatildi/sifirlandi.')

    # ======================= RC CIKISI (50 Hz) ==========================
    def publish_rc_logic(self):
        # Kullanicidan girdi beklenirken hareket etme
        if self.hedef_bekleniyor or self.sekil_bekleniyor:
            return
        # GUIDED: hareketi setpoint_position sagliyor, RC override yok
        if self.mod == 'GUIDED':
            return

        if self.mod == 'SEKIL':
            # Ulasildiysa RC override gonderme; arac ArduPilot HOLD'da bekler
            if self.sekil_tamamlandi:
                return
            gaz, dumen = self.sekil_servo_hesapla()
        else:  # MANUAL
            sol = self.sol_durum * self.hiz
            sag = self.sag_durum * self.hiz
            gaz = (sol + sag) / 2.0
            dumen = ((sol - sag) / 2.0) * DUMEN_YONU

        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18
        rc_msg.channels[KANAL_DUMEN] = self.calculate_pwm(dumen)
        rc_msg.channels[KANAL_GAZ] = self.calculate_pwm(gaz)
        self.rc_pub.publish(rc_msg)

    # ==================== SEKIL GORSEL SERVO =============================
    def sekil_servo_hesapla(self):
        with self.sekil_lock:
            gorunuyor = self.sekil_gorunuyor
            onaylandi = self.sekil_onaylandi
            cX = self.sekil_cX
            cY = self.sekil_cY
            taze = (time.time() - self.sekil_dist_time) < DEPTH_TIMEOUT
            dist = self.sekil_dist if taze else 0.0

        # Onay yoksa veya hedef gorunmuyorsa GUVENLIK icin dur
        if not onaylandi or not gorunuyor:
            self.sekil_reached_t0 = None
            return 0.0, 0.0

        # Yatay hata (normalize): + ise hedef SAGDA
        err_x = (cX - (PREVIEW_W / 2.0)) / (PREVIEW_W / 2.0)

        # DIREKSIYON: hedefe don (sag hedef -> saga don)
        dumen = max(-MAX_STEER, min(MAX_STEER, KP_STEER * err_x)) * DUMEN_YONU

        # ILERI/GERI: mesafe kontrolu (hedef 1 m)
        if dist <= 0.0:
            gaz = 0.0  # gecerli derinlik yok -> guvenlik icin ilerleme
        else:
            hata = dist - HEDEF_MESAFE
            if hata > MESAFE_TOL:
                gaz = max(0.0, min(MAX_YAKLASMA_HIZI, KP_MESAFE * hata))
                gaz *= max(0.0, 1.0 - abs(err_x) * HIZ_KESME)   # merkezde degilken once don
            elif hata < -MESAFE_TOL:
                gaz = max(-MAX_YAKLASMA_HIZI, min(0.0, KP_MESAFE * hata))  # cok yakin -> geri
            else:
                gaz = 0.0

        # ULASILDI mi? (yatayda merkez + ~1 m mesafe, ULASILDI_SURE boyunca)
        hedefte = (dist > 0.0 and abs(dist - HEDEF_MESAFE) <= MESAFE_TOL and abs(err_x) <= MERKEZ_TOL)
        if hedefte:
            if self.sekil_reached_t0 is None:
                self.sekil_reached_t0 = time.time()
            elif time.time() - self.sekil_reached_t0 >= ULASILDI_SURE:
                self.sekil_tamamlandi = True
                self.sol_durum = self.sag_durum = 0
                req = SetMode.Request(); req.custom_mode = 'HOLD'; self.mode_client.call_async(req)
                print(f"\n[SEKIL] HEDEF ({self.target_color} {self.target_shape}) MERKEZLENDI ve "
                      f"~1 m mesafeye ULASILDI. Motorlar durduruldu (HOLD). Yeni sekil icin 'k'.")
                self.sekil_durum_yayinla("HEDEF_ULASILDI", cX, cY, dist)
                if POMPA_OTOMATIK_ATES:
                    self.pompa_ates()
                return 0.0, 0.0
        else:
            self.sekil_reached_t0 = None

        # Terminal durum (throttle'li)
        now = time.time()
        if now - self.sekil_last_print > 0.5:
            print(f"\r[SEKIL TAKIP] {self.target_color} {self.target_shape} | "
                  f"ofset:{err_x:+.2f} | mesafe:{dist:.2f} m | gaz:{gaz:+.2f} dumen:{dumen:+.2f}   ",
                  end="", flush=True)
            self.sekil_last_print = now

        return gaz, dumen

    def sekil_durum_yayinla(self, durum, x, y, dist):
        msg = String()
        msg.data = (f"{durum} | {self.target_color} {self.target_shape} | "
                    f"X:{x},Y:{y} | D:{dist:.2f}m | C:{self.confirmation_count}")
        self.detect_pub.publish(msg)

    # ======================= POMPA (role) =============================
    def pompa_ates(self):
        """Hedefe ulasilinca bir kez tetiklenir: rolayi acar, POMPA_SU_SIKMA_SURESI
        saniye sonra ayri bir zamanlayici thread'i otomatik kapatir. RC dongusunu
        (50 Hz) bloklamamak icin time.sleep KULLANILMAZ."""
        if self.pompa_atildi:
            return
        self.pompa_atildi = True
        self.arduino_yaz(POMPA_KOMUT_AC)
        print(f"\n[POMPA] ATESLENDI ({POMPA_SU_SIKMA_SURESI:.1f} sn sonra otomatik kapanacak).")
        if self.pompa_timer is not None:
            self.pompa_timer.cancel()
        self.pompa_timer = threading.Timer(POMPA_SU_SIKMA_SURESI, self.pompa_kapat)
        self.pompa_timer.daemon = True
        self.pompa_timer.start()

    def pompa_kapat(self):
        self.arduino_yaz(POMPA_KOMUT_KAPAT)
        print("\n[POMPA] KAPATILDI.")

    # ======================= GPS (waypoint) =============================
    def gps_callback(self, msg):
        if msg.status.status < 0:
            self.get_logger().warn('GPS fix yok, bekleniyor...', throttle_duration_sec=5.0)
            return

        if not self.is_calibrated:
            self.kal_lat_buf.append(msg.latitude)
            self.kal_lon_buf.append(msg.longitude)
            if len(self.kal_lat_buf) >= KALIBRASYON_N:
                self.ref_lat = statistics.median(self.kal_lat_buf)
                self.ref_lon = statistics.median(self.kal_lon_buf)
                self.is_calibrated = True
                self.get_logger().info('\n GPS REFERANSI ALINDI.')
                self.hedef_bekleniyor = True
            return

        R = 6371000.0
        raw_n = R * math.radians(msg.latitude - self.ref_lat)
        raw_e = R * math.cos(math.radians(self.ref_lat)) * math.radians(msg.longitude - self.ref_lon)

        self.ema_n_v = raw_n if self.ema_n_v is None else (EMA_ALPHA * raw_n + (1 - EMA_ALPHA) * self.ema_n_v)
        self.ema_e_v = raw_e if self.ema_e_v is None else (EMA_ALPHA * raw_e + (1 - EMA_ALPHA) * self.ema_e_v)

        dist = math.sqrt((self.hedef_n - self.ema_n_v) ** 2 + (self.hedef_e - self.ema_e_v) ** 2)

        # Sadece GUIDED modda GPS takibi yap
        if self.mod == 'GUIDED' and not self.hedef_bekleniyor:
            if dist < HEDEF_TOLERANSI and not self.reached_target:
                self.reached_target = True
                req = SetMode.Request(); req.custom_mode = 'HOLD'; self.mode_client.call_async(req)
                print(f"\n {self.target_lat:.6f}, {self.target_lon:.6f} HEDEFINE ULASILDI! Yeni rota icin 'h'.")

            if not self.reached_target:
                print(f"\r[OTONOM] Mevcut: {msg.latitude:.6f},{msg.longitude:.6f} | "
                      f"Hedefe Kalan: {dist:.1f} m    ", end="", flush=True)
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(self.hedef_e)
                pose.pose.position.y = float(self.hedef_n)
                self.pose_pub.publish(pose)
            else:
                print(f"\r[HEDEFTE BEKLIYOR] Mesafe: {dist:.1f} m    ", end="", flush=True)

    # ==================== KULLANICI GIRISLERI ===========================
    def kullanici_hedef_al(self):
        # Terminal NORMAL moddayken cagrilir
        req = SetMode.Request(); req.custom_mode = 'HOLD'; self.mode_client.call_async(req)
        try:
            print("\n" + "=" * 62)
            girdi = input("Hedef Gir (Enlem Boylam) [Orn: 41.12345 29.12345]: ")
            lat, lon = map(float, girdi.split())
            self.target_lat, self.target_lon = lat, lon
            R = 6371000.0
            self.hedef_n = R * math.radians(lat - self.ref_lat)
            self.hedef_e = R * math.cos(math.radians(self.ref_lat)) * math.radians(lon - self.ref_lon)
            self.reached_target = False
            self.mod = 'GUIDED'
            req = SetMode.Request(); req.custom_mode = 'GUIDED'; self.mode_client.call_async(req)
            print("=" * 62 + "\n")
        except Exception:
            print("\n[HATA] Gecersiz giris! 'h' ile tekrar deneyin.")
        finally:
            self.hedef_bekleniyor = False

    def kullanici_sekil_al(self):
        # Terminal NORMAL moddayken cagrilir
        try:
            print("\n" + "=" * 62)
            print(" SEKIL TESPIT MODU (SU SIKMA GOREVI)")
            print("   Renkler : Kirmizi / Yesil / Sari / Siyah")
            print("   Sekiller: GEMI")
            girdi = input(" HEDEF (Renk Sekil) [Orn: Kirmizi GEMI]: ").strip().split()
            # ... (fonksiyonun geri kalanı aynı kalacak)
            if len(girdi) == 2:
                self.target_color = girdi[0].capitalize()
                self.target_shape = girdi[1].upper()
                # Tespit durumunu sifirla
                with self.sekil_lock:
                    self.confirmation_count = 0
                    self.sekil_onaylandi = False
                    self.sekil_gorunuyor = False
                    self.sekil_tamamlandi = False
                    self.sekil_reached_t0 = None
                    self.sekil_dist = 0.0
                    self.sekil_dist_time = 0.0
                # Yeni hedef icin pompayi guvenlik amacli kapat ve bayragi sifirla
                if self.pompa_timer is not None:
                    self.pompa_timer.cancel()
                if self.pompa_atildi:
                    self.pompa_kapat()
                self.pompa_atildi = False
                # RC override'in motoru surebilmesi icin ArduPilot MANUAL
                req = SetMode.Request(); req.custom_mode = 'MANUAL'; self.mode_client.call_async(req)
                print(f" Hedef ayarlandi: {self.target_color} {self.target_shape}")
                print(" ArduPilot MANUAL'e alindi. Motorlar icin ARM (t) gerektigini unutma.")
                print(f" Sekil {ONAY_KARE} kare onaylaninca otonom yaklasma baslar.")
                print("=" * 62 + "\n")
            else:
                print(" [HATA] Iki deger gir: 'Renk Sekil'. Iptal edildi.")
                self.mod = 'MANUAL'
        except Exception:
            print(" [HATA] Gecersiz giris. Iptal edildi.")
            self.mod = 'MANUAL'
        finally:
            self.sekil_bekleniyor = False


# ==================== KAMERA / SEKIL TESPIT DONGUSU =========================
def sekil_vision_loop(node):
    try:
        pipeline = dai.Pipeline()
        
        # --- KAMERA PORT/SOCKET ATAMALARI EKLENDİ ---
        
        # 1. RGB Kamera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A) # EKSİKTİ: Modern sürümlerde ZORUNLU
        cam_rgb.setPreviewSize(PREVIEW_W, PREVIEW_H)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)

        # 2. Stereo Depth ve Mono Kameralar
        stereo = pipeline.create(dai.node.StereoDepth)
        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_r = pipeline.create(dai.node.MonoCamera)
        
        mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B) # EKSİKTİ: Sol kamera portu
        mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C) # EKSİKTİ: Sağ kamera portu
        
        mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY) # Daha stabil derinlik için eklendi
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        
        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)

        # 3. XLink Çıkışları
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)
        
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        with dai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            node.get_logger().info(' OAK-D kamera hazir - sekil tespit calisiyor.')

            while rclpy.ok() and not node.kapat:
                in_rgb = q_rgb.get()
                in_depth = q_depth.get()
                if in_rgb is None or in_depth is None:
                    continue

                # Bassiz calisirken CPU tasarrufu: yalniz SEKIL modunda agir tespit yap
                if not SHOW_WINDOW and node.mod != 'SEKIL':
                    continue

                frame = in_rgb.getCvFrame()
                depth_frame = in_depth.getFrame()
                output = frame.copy()
                hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

                # --- RENK MASKELERI ---
                mask_red = cv2.addWeighted(
                    cv2.inRange(hsv, np.array([0, 140, 60]), np.array([10, 255, 255])), 1.0,
                    cv2.inRange(hsv, np.array([160, 140, 60]), np.array([180, 255, 255])), 1.0, 0.0)
                mask_green = cv2.inRange(hsv, np.array([35, 40, 30]), np.array([95, 255, 255]))
                mask_yellow = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([35, 255, 255]))
                mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 120, 35]))

                colors = [("Yesil", mask_green, (0, 255, 0)),
                          ("Kirmizi", mask_red, (0, 0, 255)),
                          ("Sari", mask_yellow, (0, 255, 255)),
                          ("Siyah", mask_black, (40, 40, 40))]

                frame_target_found = False
                best = None  # (area, cX, cY, dist_m, approx, box)

                for c_name, mask, bgr in colors:
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for cnt in contours:
                        shape, approx, def_c, sol, circ, ext = get_shape_name(cnt)
                        x_b, y_b, w_b, h_b = cv2.boundingRect(cnt)
                        if cv2.contourArea(cnt) > 500:
                            cv2.putText(output, f"D:{def_c} S:{sol}", (x_b, y_b + h_b + 12),
                                        FONT, 0.4, (0, 255, 255), 1)
                        if shape:
                            label = f"{c_name} {shape}"
                            cv2.rectangle(output, (x_b, y_b), (x_b + w_b, y_b + h_b), bgr, 2)
                            cv2.putText(output, label, (x_b, y_b - 5), FONT, 0.5, bgr, 2)
                            # Sadece istenen hedefi takip et (en buyuk eslesme)
                            if c_name == node.target_color and shape == node.target_shape:
                                cX, cY = x_b + w_b // 2, y_b + h_b // 2
                                dmm = depth_ortala(depth_frame, cX, cY)
                                area = cv2.contourArea(cnt)
                                if best is None or area > best[0]:
                                    best = (area, cX, cY, dmm / 1000.0 if dmm > 0 else 0.0,
                                            approx, (x_b, y_b, w_b, h_b))
                                frame_target_found = True

                # --- Onay + paylasilan durum guncelle ---
                if frame_target_found and best is not None:
                    area, cX, cY, dist_m, approx, box = best
                    node.confirmation_count += 1
                    with node.sekil_lock:
                        node.sekil_cX = cX
                        node.sekil_cY = cY
                        if dist_m > 0:
                            node.sekil_dist = dist_m
                            node.sekil_dist_time = time.time()
                        node.sekil_gorunuyor = True
                    if node.confirmation_count >= node.required_frames:
                        node.sekil_onaylandi = True
                        x_b, y_b, w_b, h_b = box
                        cv2.drawContours(output, [approx], -1, (0, 255, 0), 3)
                        durum = "TAMAMLANDI" if node.sekil_tamamlandi else "TAKIP EDILIYOR"
                        cv2.putText(output, f"HEDEF {durum}", (x_b, y_b - 25), FONT, 0.7, (0, 255, 0), 2)
                        node.sekil_durum_yayinla("HEDEF_ONAYLANDI", cX, cY, dist_m)
                    else:
                        node.sekil_durum_yayinla("ONAY_BEKLENIYOR", cX, cY, dist_m)
                else:
                    node.confirmation_count = 0
                    with node.sekil_lock:
                        node.sekil_gorunuyor = False

                # --- Ekran bilgilendirme ---
                if SHOW_WINDOW:
                    cx_c = PREVIEW_W // 2
                    cv2.line(output, (cx_c, 0), (cx_c, PREVIEW_H), (255, 255, 255), 1)
                    cv2.putText(output, f"MOD: {node.mod}  HEDEF: {node.target_color} {node.target_shape}",
                                (10, 25), FONT, 0.6, (255, 255, 0), 2)
                    if node.sekil_onaylandi:
                        d = node.sekil_dist if (time.time() - node.sekil_dist_time) < DEPTH_TIMEOUT else 0.0
                        cv2.putText(output, f"Mesafe: {d:.2f} m (hedef {HEDEF_MESAFE:.1f} m)",
                                    (10, 50), FONT, 0.6, (0, 255, 0), 2)
                    cv2.imshow("ITU AS - Sekil + Waypoint", output)
                    if (cv2.waitKey(1) & 0xFF) == ord('q'):
                        node.kapat = True
                        break

        if SHOW_WINDOW:
            cv2.destroyAllWindows()

    except Exception as e:
        # Kamera acilmazsa: sekil modu devre disi, GPS/MANUAL calismaya devam etsin
        node.kamera_var = False
        node.get_logger().error(f' OAK-D baslatilamadi ({e}). SEKIL modu devre disi; GPS/MANUAL calisir.')
        while rclpy.ok() and not node.kapat:
            time.sleep(0.5)


# ================================ MAIN ======================================
def main(args=None):
    rclpy.init(args=args)
    node = EntegreNode()

    # ROS'u arka planda dondur (timer + abonelikler); kamera dongusu ANA thread'de
    # (OpenCV GUI'nin ana thread'de olmasi gerektigi icin bu duzen tercih edildi)
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        sekil_vision_loop(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Kamera dongusu hatasi: {e}')
    finally:
        node.kapat = True
        # Guvenlik: motorlari notrle
        try:
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[KANAL_DUMEN] = 1500
            rc_msg.channels[KANAL_GAZ] = 1500
            node.rc_pub.publish(rc_msg)
        except Exception:
            pass
        # Guvenlik: pompayi kapat ve Arduino baglantisini serbest birak
        try:
            if node.pompa_timer is not None:
                node.pompa_timer.cancel()
            node.arduino_yaz(POMPA_KOMUT_KAPAT)
            if node.arduino is not None:
                node.arduino.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
