#!/usr/bin/env python3
"""
Njord — Basit Kayıt + Klavye Sürüş Scripti (ROS YOK)

OAK-D kamerasından 640x640 mp4 kaydı alır; aynı anda Pixhawk'a doğrudan MAVLink
(pymavlink) ile bağlanıp klavyeden ARM/DISARM/MANUAL mod + gaz/dümen sürüşü sağlar.
Tamamen BAĞIMSIZ tek dosyadır — ROS/rclpy YOK, başka bir proje dosyasına (config/ortak
modül) ihtiyaç duymaz; PWM/mixing mantığı (calculate_pwm, mix_gaz_dumen) ve motor
ayarları (KANAL_DUMEN, KANAL_GAZ, RC_PWM_*) doğrudan bu dosyanın içinde tanımlı.

Bağımlılıklar (Jetson'da): pip install depthai opencv-python pymavlink numpy

Kontroller:
  Ok tuşları : yukarı/aşağı = gaz, sağ/sol = dümen (tuşu bırakınca ~DRIVE_DEADMAN_S
               içinde otomatik nötre döner — dead-man switch, kontrolsüz sabit gazda
               kalmayı önler)
  a          : ARM
  x          : DISARM + motor nötr
  m          : MANUAL mod iste (way.py'deki gibi — RC-override'ın etkili olduğu mod)
  v          : Kayıt başlat/durdur (her başlatmada yeni zaman damgalı .mp4)
  q          : Çıkış (override serbest bırakılır, dosya düzgün kapatılır)
"""

import os
import sys
import time
import threading
from datetime import datetime

import cv2
import depthai as dai
from pymavlink import mavutil

try:
    import termios
    import tty
except ImportError:  # Windows gibi tty'siz ortamlarda klavye dinleyici pasif kalır
    termios = None
    tty = None

# --- AYARLAR (Jetson'daki gerçek bağlantıya göre düzenle) ---
PIXHAWK_PORT = '/dev/ttyACM0'   # USB-seri direkt bağlantı; telemetri radyosu kullanılıyorsa
PIXHAWK_BAUD = 115200           # değiştir (örn. /dev/ttyUSB0, baud 57600)
CAM_SIZE = (640, 640)
CAM_FPS = 30
DRIVE_STEP = 0.15               # ok tuşu başına gaz/dümen eksenindeki artış
DRIVE_DEADMAN_S = 0.3           # bu süre yeni tuş gelmezse otomatik nötr
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- MOTOR ÇIKIŞI AYARLARI (eskiden njord_config.json/oak_common.py'de idi, bu script
# artık tamamen bağımsız olduğu için buraya gömüldü) ---
KANAL_DUMEN = 0                 # CH1 — ArduPilot'un kendi skid-steer mixer'i alıyor
KANAL_GAZ = 2                   # CH3
DUMEN_YONU = 1                  # sol/sağ motorlar ters çalışırsa -1 yapın
RC_PWM_CENTER = 1500
RC_PWM_SCALE = 400
RC_PWM_DEADZONE = 30            # bu kadar merkeze yakınsa tam nötr (1500) kabul edilir
RC_PWM_MIN = 1100
RC_PWM_MAX = 1900


def calculate_pwm(axis_val: float, center: int = RC_PWM_CENTER, scale: int = RC_PWM_SCALE,
                   deadzone: int = RC_PWM_DEADZONE, pwm_min: int = RC_PWM_MIN,
                   pwm_max: int = RC_PWM_MAX) -> int:
    """[-1,1] civarı bir eksen değerini PWM'e çevirir, merkeze yakınsa tam nötr döner,
    sonucu [pwm_min, pwm_max] aralığına kırpar."""
    val = center + int(axis_val * scale)
    if abs(val - center) < deadzone:
        return center
    return max(pwm_min, min(pwm_max, val))


def mix_gaz_dumen(sol: float, sag: float, dumen_yonu: int = DUMEN_YONU) -> tuple:
    """İstenen sol/sağ motor değerlerinden (ArduPilot'un kendi skid-steer mixer'inin
    CH1/CH3'ten beklediği) gaz/dümen eksenlerini üretir. Döner: (gaz, dumen)."""
    gaz = (sol + sag) / 2.0
    dumen = ((sol - sag) / 2.0) * dumen_yonu
    return gaz, dumen


class MavlinkMotor:
    """Pixhawk'a doğrudan MAVLink bağlantısı — mavros/ROS YOK. ARM/DISARM/MANUAL mod
    çağrıları fire-and-forget'tır (way.py'deki deseninin ruhu aynı: bloklamadan
    gönderilir); güvenlik açısından kritik olan ACK'i beklemek değil, override'ın
    PERİYODİK gönderilmesidir — kesilirse ArduPilot bir süre sonra kontrolü zaten
    RC vericisine geri verir (RC_OVERRIDE_TIME parametresi)."""

    def __init__(self, port: str, baud: int):
        print(f"[MAVLink] Bağlanılıyor: {port} @ {baud}...")
        self.master = mavutil.mavlink_connection(port, baud=baud)
        if self.master.wait_heartbeat(timeout=10) is None:
            raise RuntimeError(
                f"Pixhawk'tan heartbeat alınamadı ({port} @ {baud}) — bağlantıyı/portu kontrol et.")
        print(f"[MAVLink] Heartbeat alındı (sys={self.master.target_system}, "
              f"comp={self.master.target_component})")
        self._lock = threading.Lock()

    def arm(self, value: bool):
        with self._lock:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1 if value else 0, 0, 0, 0, 0, 0, 0)
        print(f"[MAVLink] {'ARM' if value else 'DISARM'} komutu gönderildi.")

    def set_mode(self, mode_name: str):
        mapping = self.master.mode_mapping()
        if not mapping or mode_name not in mapping:
            print(f"[MAVLink] ⚠️  Mod '{mode_name}' bulunamadı "
                  f"(autopilot tipi henüz tanınmamış olabilir).")
            return
        with self._lock:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[mode_name])
        print(f"[MAVLink] Mod isteği gönderildi: {mode_name}")

    def send_override(self, dumen_pwm: int, gaz_pwm: int):
        """chan1..chan8 gönderir — sadece KANAL_DUMEN/KANAL_GAZ indeksleri set edilir,
        diğerleri UINT16_MAX (65535) = 'bu kanala dokunma' (MAVLink RC_CHANNELS_OVERRIDE
        spesifikasyonundaki sentinel değeri)."""
        chans = [65535] * 8
        chans[KANAL_DUMEN] = dumen_pwm
        chans[KANAL_GAZ] = gaz_pwm
        with self._lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                chan1_raw=chans[0], chan2_raw=chans[1], chan3_raw=chans[2], chan4_raw=chans[3],
                chan5_raw=chans[4], chan6_raw=chans[5], chan7_raw=chans[6], chan8_raw=chans[7])

    def release_override(self):
        """Çıkışta override'ı BIRAKIR (0 = kanalı RC vericisine geri ver) — nötr PWM
        göndermek yerine kontrolü tamamen pilota/vericiye teslim eder."""
        chans = [65535] * 8
        chans[KANAL_DUMEN] = 0
        chans[KANAL_GAZ] = 0
        with self._lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                chan1_raw=chans[0], chan2_raw=chans[1], chan3_raw=chans[2], chan4_raw=chans[3],
                chan5_raw=chans[4], chan6_raw=chans[5], chan7_raw=chans[6], chan8_raw=chans[7])


class DriveState:
    """Ok tuşlarıyla değiştirilen gaz/dümen ekseni [-1,1] — dead-man switch:
    DRIVE_DEADMAN_S içinde yeni tuş gelmezse read_and_decay() tarafından nötre
    çekilir (elden bırakınca kontrolsüz sabit gaz/dümende kalınmasın diye)."""

    def __init__(self):
        self.throttle = 0.0
        self.steer = 0.0
        self.last_key_time = 0.0
        self._lock = threading.Lock()

    def nudge(self, dthrottle: float = 0.0, dsteer: float = 0.0):
        with self._lock:
            self.throttle = max(-1.0, min(1.0, self.throttle + dthrottle))
            self.steer = max(-1.0, min(1.0, self.steer + dsteer))
            self.last_key_time = time.time()

    def read_and_decay(self) -> tuple:
        with self._lock:
            if (time.time() - self.last_key_time) > DRIVE_DEADMAN_S:
                self.throttle = 0.0
                self.steer = 0.0
            return self.throttle, self.steer

    def zero(self):
        with self._lock:
            self.throttle = 0.0
            self.steer = 0.0
            self.last_key_time = 0.0


def _drive_loop(motor: MavlinkMotor, drive: DriveState, stop_event: threading.Event):
    """10Hz'de sürekli RC override gönderir — tuşa basılmasa bile bu döngü durmamalı,
    aksi halde ArduPilot override'ı zaman aşımına uğratıp kontrolü RC vericisine
    (ya da hiçbir şeye) geri verebilir, sürüş sessizce durur."""
    while not stop_event.is_set():
        throttle, steer = drive.read_and_decay()
        sol = max(-1.0, min(1.0, throttle - steer))
        sag = max(-1.0, min(1.0, throttle + steer))
        gaz, dumen = mix_gaz_dumen(sol, sag)
        motor.send_override(calculate_pwm(dumen), calculate_pwm(gaz))
        time.sleep(0.1)


class Recorder:
    """cv2.VideoWriter etrafında ince bir sarmalayıcı — 'v' tuşuna her basışta yeni
    zaman damgalı bir .mp4 dosyası açar/kapatır."""

    def __init__(self, size: tuple, fps: int, out_dir: str):
        self.size = size
        self.fps = fps
        self.out_dir = out_dir
        self.writer = None
        self._lock = threading.Lock()

    def toggle(self):
        with self._lock:
            if self.writer is not None:
                self.writer.release()
                self.writer = None
                print("[Kayıt] ⏹ durduruldu.")
                return
            path = os.path.join(self.out_dir, f"kayit_{datetime.now():%Y%m%d_%H%M%S}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(path, fourcc, self.fps, self.size)
            print(f"[Kayıt] ⏺ başladı: {path}")

    def write(self, frame):
        with self._lock:
            if self.writer is not None:
                self.writer.write(frame)

    def close(self):
        with self._lock:
            if self.writer is not None:
                self.writer.release()
                self.writer = None


def _keypress_listener(motor: MavlinkMotor, drive: DriveState, recorder: Recorder,
                        stop_event: threading.Event):
    """Ok tuşları '\\x1b[A/B/C/D' üç-baytlık escape dizisi olarak gelir — ilk bayt ESC
    ise kalan iki baytı ayrıca okuyup yön çıkarıyoruz."""
    if termios is None or tty is None or not sys.stdin.isatty():
        print("⚠️  Klavye dinleyici başlatılamadı (tty yok) — sadece kayıt/otomatik "
              "davranışla çalışılacak, motor/kayıt tuşları pasif.")
        return
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                seq = sys.stdin.read(2)
                if seq == '[A':
                    drive.nudge(dthrottle=DRIVE_STEP)
                elif seq == '[B':
                    drive.nudge(dthrottle=-DRIVE_STEP)
                elif seq == '[C':
                    drive.nudge(dsteer=DRIVE_STEP)
                elif seq == '[D':
                    drive.nudge(dsteer=-DRIVE_STEP)
            elif ch == 'a':
                motor.arm(True)
            elif ch == 'x':
                drive.zero()
                motor.arm(False)
            elif ch == 'm':
                motor.set_mode('MANUAL')
            elif ch == 'v':
                recorder.toggle()
            elif ch == 'q':
                stop_event.set()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _build_camera_queue():
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setPreviewSize(*CAM_SIZE)
    cam.setInterleaved(False)
    # depthai sürümüne göre enum adı değişti (eski: RGB, yeni: CAM_A) — hangisi
    # varsa onu kullan.
    socket = getattr(dai.CameraBoardSocket, 'CAM_A', None) or dai.CameraBoardSocket.RGB
    cam.setBoardSocket(socket)
    cam.setFps(CAM_FPS)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName('video')
    cam.preview.link(xout.input)

    device = dai.Device(pipeline)
    return device, device.getOutputQueue('video', maxSize=4, blocking=False)


def main():
    motor = MavlinkMotor(PIXHAWK_PORT, PIXHAWK_BAUD)
    drive = DriveState()
    recorder = Recorder(CAM_SIZE, CAM_FPS, OUTPUT_DIR)
    stop_event = threading.Event()

    device, q = _build_camera_queue()
    print("✅ OAK-D kamera hazır.")

    drive_thread = threading.Thread(target=_drive_loop, args=(motor, drive, stop_event), daemon=True)
    key_thread = threading.Thread(
        target=_keypress_listener, args=(motor, drive, recorder, stop_event), daemon=True)
    drive_thread.start()
    key_thread.start()

    print(
        "\n[Ok tuşları] Gaz/Dümen   [a] ARM   [x] DISARM+nötr   [m] MANUAL mod   "
        "[v] Kayıt başlat/durdur   [q] Çıkış\n"
    )

    try:
        while not stop_event.is_set():
            frame = q.get().getCvFrame()
            recorder.write(frame)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        drive.zero()
        motor.release_override()
        recorder.close()
        device.close()
        print("👋 Güvenli kapanış tamamlandı.")


if __name__ == '__main__':
    main()
