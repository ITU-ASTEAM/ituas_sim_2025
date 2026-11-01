#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SSH / TERMİNAL SÜRÜMÜ — harici kütüphane ve /dev/input gerektirmez.
# Tuşlar terminalden (stdin) okunur, SSH/WiFi üzerinden çalışır.
# TOGGLE MANTIĞI: tuşa bir kez bas -> motor başlar, tekrar bas -> durur.
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
import math
import statistics
import threading
import sys
import termios
import tty
import select

# --- AYARLAR ---
KALIBRASYON_N = 10
EMA_ALPHA = 0.12
HEDEF_TOLERANSI = 1.2
VARSAYILAN_HIZ = 0.5      # 0.0 - 1.0 arasi motor gucu

# --- RC KANALLARI (ArduPilot karışımı KENDİSİ yapıyor: SERVOx=ThrottleLeft/Right) ---
KANAL_DUMEN = 0   # CH1 -> Direksiyon girişi
KANAL_GAZ = 2     # CH3 -> Gaz girişi
DUMEN_YONU = 1    # Sol/sağ motorlar ters çalışırsa bunu -1 yap

# --- TUŞ HARİTASI (TOGGLE: bas=başla, tekrar bas=dur) ---
#  7 : SOL motor İLERİ        9 : SAĞ motor İLERİ
#  1 : SOL motor GERİ         3 : SAĞ motor GERİ
#  w : ikisi İLERİ            s : ikisi GERİ
#  a : sola pivot (sol geri + sağ ileri)
#  d : sağa pivot (sol ileri + sağ geri)
#  5 veya SPACE : TÜM MOTORLAR DUR
#  + / - : hız artır / azalt
#  t : ARM    y : DISARM    m : MANUAL    g : GUIDED
#  h : yeni hedef gir

class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_generator')

        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile_sensor_data)

        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.timer = self.create_timer(0.02, self.publish_rc_logic)  # 50 Hz

        self.is_calibrated = False
        self.autonomous_enabled = True
        self.reached_target = False
        self.ref_lat, self.ref_lon = None, None

        self.target_lat, self.target_lon = 0.0, 0.0
        self.hedef_n, self.hedef_e = 0.0, 0.0

        self.ema_n_v, self.ema_e_v = None, None
        self.kal_lat_buf, self.kal_lon_buf = [], []

        self.hedef_bekleniyor = False
        self.hiz = VARSAYILAN_HIZ

        # --- MOTOR DURUMLARI (toggle): -1 geri, 0 dur, +1 ileri ---
        self.sol_durum = 0
        self.sag_durum = 0

        # Terminal (stdin) dinleyici thread'i
        self.klavye_thread = threading.Thread(target=self.klavye_dongusu, daemon=True)
        self.klavye_thread.start()

        self.get_logger().info('🚀 SİSTEM BAŞLATILDI: SSH Terminal Kontrolü (Toggle) Aktif')
        self.yardim_yazdir()

    def yardim_yazdir(self):
        print("\n" + "=" * 62)
        print(" MOTOR KONTROLLERİ — TOGGLE (bas: başla, tekrar bas: dur)")
        print("   7 : SOL İLERİ    9 : SAĞ İLERİ")
        print("   1 : SOL GERİ     3 : SAĞ GERİ")
        print("   w : ikisi ileri  s : ikisi geri")
        print("   a : sola pivot   d : sağa pivot")
        print("   5 / SPACE : TÜM MOTORLAR DUR")
        print(" DİĞER")
        print("   +/- : hız (mevcut %.1f)" % self.hiz)
        print("   t : ARM    y : DISARM    m : MANUAL   g : GUIDED")
        print("   h : yeni hedef gir")
        print(" NOT: Toggle olduğu için motorlar sen durdurana kadar döner.")
        print("      Acil durumda 5, SPACE veya y (DISARM)!")
        print("=" * 62 + "\n")

    def calculate_pwm(self, axis_val):
        val = 1500 + int(axis_val * 400.0)
        if abs(val - 1500) < 30:
            return 1500
        return max(1100, min(1900, val))

    def durum_yaz(self):
        isim = {-1: 'GERİ ', 0: 'DUR  ', 1: 'İLERİ'}
        print(f"\n[MOTOR] SOL: {isim[self.sol_durum]} | SAĞ: {isim[self.sag_durum]} | Hız: {self.hiz:.1f}")

    # ------------------- TERMİNAL DİNLEYİCİ -------------------
    def klavye_dongusu(self):
        self.term_ayarlari = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                if self.hedef_bekleniyor:
                    self.kullanici_hedef_al()
                    continue

                tty.setcbreak(sys.stdin.fileno())
                okunabilir, _, _ = select.select([sys.stdin], [], [], 0.1)
                tus = sys.stdin.read(1) if okunabilir else ''
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_ayarlari)

                if tus:
                    self.tus_isle(tus.lower())
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_ayarlari)

    def tus_isle(self, t):
        # --- MOTOR TOGGLE TUŞLARI ---
        if t == '7':      # sol ileri (tekrar basılırsa dur)
            self.sol_durum = 1 if self.sol_durum != 1 else 0
            self.durum_yaz()
        elif t == '1':    # sol geri
            self.sol_durum = -1 if self.sol_durum != -1 else 0
            self.durum_yaz()
        elif t == '9':    # sağ ileri
            self.sag_durum = 1 if self.sag_durum != 1 else 0
            self.durum_yaz()
        elif t == '3':    # sağ geri
            self.sag_durum = -1 if self.sag_durum != -1 else 0
            self.durum_yaz()

        # --- ÇİFT MOTOR KISAYOLLARI (toggle) ---
        elif t == 'w':    # ikisi ileri
            hedef = 0 if (self.sol_durum, self.sag_durum) == (1, 1) else 1
            self.sol_durum = self.sag_durum = hedef
            self.durum_yaz()
        elif t == 's':    # ikisi geri
            hedef = 0 if (self.sol_durum, self.sag_durum) == (-1, -1) else -1
            self.sol_durum = self.sag_durum = hedef
            self.durum_yaz()
        elif t == 'a':    # sola pivot: sol geri, sağ ileri
            if (self.sol_durum, self.sag_durum) == (-1, 1):
                self.sol_durum = self.sag_durum = 0
            else:
                self.sol_durum, self.sag_durum = -1, 1
            self.durum_yaz()
        elif t == 'd':    # sağa pivot: sol ileri, sağ geri
            if (self.sol_durum, self.sag_durum) == (1, -1):
                self.sol_durum = self.sag_durum = 0
            else:
                self.sol_durum, self.sag_durum = 1, -1
            self.durum_yaz()

        elif t in (' ', '5'):
            self.sol_durum = self.sag_durum = 0
            print("\n🛑 TÜM MOTORLAR DUR (1500).")

        # --- HIZ ---
        elif t == '+' or t == '=':
            self.hiz = min(1.0, self.hiz + 0.1)
            print(f"\n⚡ Hız: {self.hiz:.1f}")
        elif t == '-':
            self.hiz = max(0.1, self.hiz - 0.1)
            print(f"\n🐢 Hız: {self.hiz:.1f}")

        # --- ARM / DISARM ---
        elif t == 't':
            req = CommandBool.Request(); req.value = True; self.arm_client.call_async(req)
            self.get_logger().info('🔒 ARM İsteği Gönderildi.')
        elif t == 'y':
            self.sol_durum = self.sag_durum = 0
            req = CommandBool.Request(); req.value = False; self.arm_client.call_async(req)
            self.get_logger().warn('🔓 DISARM İsteği Gönderildi.')

        # --- MODLAR ---
        elif t == 'm':
            self.autonomous_enabled = False
            self.sol_durum = self.sag_durum = 0  # manuel moda nötr başla
            req = SetMode.Request(); req.custom_mode = 'MANUAL'; self.mode_client.call_async(req)
            self.get_logger().warn('\n🚨 MOD: MANUAL (TUŞ SÜRÜŞÜ)')
        elif t == 'g':
            self.autonomous_enabled = True
            self.sol_durum = self.sag_durum = 0
            req = SetMode.Request(); req.custom_mode = 'GUIDED'; self.mode_client.call_async(req)
            self.reached_target = False
            self.get_logger().info('\n🤖 MOD: GUIDED (OTONOM TAKİP)')
        elif t == 'h':
            if not self.hedef_bekleniyor and self.is_calibrated:
                self.sol_durum = self.sag_durum = 0
                self.hedef_bekleniyor = True
                self.get_logger().info('\n🔄 Yeni Hedef İsteniyor...')

    # ------------------- RC ÇIKIŞI -------------------
    def publish_rc_logic(self):
        if self.autonomous_enabled or self.hedef_bekleniyor:
            return

        # İstenen motor değerleri
        sol = self.sol_durum * self.hiz
        sag = self.sag_durum * self.hiz

        # TERS KARIŞIM: ArduPilot CH1=dümen, CH3=gaz alıp kendi
        # skid-steer karışımını yapıyor (motor = gaz ± dümen).
        # İstediğimiz sol/sağ değerlerine ulaşması için:
        gaz = (sol + sag) / 2.0
        dumen = ((sol - sag) / 2.0) * DUMEN_YONU

        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18
        rc_msg.channels[KANAL_DUMEN] = self.calculate_pwm(dumen)
        rc_msg.channels[KANAL_GAZ] = self.calculate_pwm(gaz)
        self.rc_pub.publish(rc_msg)

    # ------------------- GPS -------------------
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
                self.get_logger().info('\n✅ GPS REFERANSI ALINDI.')
                self.hedef_bekleniyor = True
            return

        R = 6371000.0
        raw_n = R * math.radians(msg.latitude - self.ref_lat)
        raw_e = R * math.cos(math.radians(self.ref_lat)) * math.radians(msg.longitude - self.ref_lon)

        self.ema_n_v = raw_n if self.ema_n_v is None else (EMA_ALPHA * raw_n + (1 - EMA_ALPHA) * self.ema_n_v)
        self.ema_e_v = raw_e if self.ema_e_v is None else (EMA_ALPHA * raw_e + (1 - EMA_ALPHA) * self.ema_e_v)

        dist = math.sqrt((self.hedef_n - self.ema_n_v)**2 + (self.hedef_e - self.ema_e_v)**2)

        if self.autonomous_enabled and not self.hedef_bekleniyor:
            if dist < HEDEF_TOLERANSI and not self.reached_target:
                self.reached_target = True
                req = SetMode.Request(); req.custom_mode = 'HOLD'; self.mode_client.call_async(req)
                print(f"\n✅ {self.target_lat:.6f}, {self.target_lon:.6f} HEDEFİNE ULAŞILDI! Yeni rota için H tuşuna basınız.")

            if not self.reached_target:
                print(f"\r[OTONOM] Mevcut: {msg.latitude:.6f}, {msg.longitude:.6f} | Hedefe Kalan: {dist:.1f}m    ", end="", flush=True)
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(self.hedef_e)
                pose.pose.position.y = float(self.hedef_n)
                self.pose_pub.publish(pose)
            else:
                print(f"\r[HEDEFTE BEKLİYOR] Mesafe: {dist:.1f}m    ", end="", flush=True)

    # ------------------- HEDEF GİRİŞİ -------------------
    def kullanici_hedef_al(self):
        # Klavye thread'i içinden, terminal NORMAL moddayken çağrılır
        req = SetMode.Request(); req.custom_mode = 'HOLD'; self.mode_client.call_async(req)
        try:
            print("\n" + "=" * 62)
            girdi = input("Hedef Gir (Enlem Boylam) [Örn: 41.12345 29.12345]: ")
            lat, lon = map(float, girdi.split())

            self.target_lat = lat
            self.target_lon = lon

            R = 6371000.0
            self.hedef_n = R * math.radians(lat - self.ref_lat)
            self.hedef_e = R * math.cos(math.radians(self.ref_lat)) * math.radians(lon - self.ref_lon)

            self.reached_target = False
            self.autonomous_enabled = True

            req = SetMode.Request(); req.custom_mode = 'GUIDED'; self.mode_client.call_async(req)
            print("=" * 62 + "\n")
        except Exception:
            print("\n[HATA] Geçersiz giriş! Lütfen H'ye basıp tekrar girin.")
        finally:
            self.hedef_bekleniyor = False


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
