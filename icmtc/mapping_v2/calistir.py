#!/usr/bin/env python3
"""
Njord Füzyon Sistemi — Ana Çalıştırma Dosyası
`lidar_fusion_node.py` (LiDAR+tracking+APF+RViz, klavye dinleyici) ve
`camera_vision_node.py` (YOLO+derinlik) dosyalarını İKİ AYRI OS process'i olarak
başlatır. Bilinçli olarak `multiprocessing.Process` (fork) DEĞİL `subprocess.Popen`
kullanılır: kamera process'i CUDA/torch context'i açtığı için fork sonrası miras
alınan bir CUDA durumu sorun çıkarabilir — her process kendi temiz Python
yorumlayıcısıyla başlamalı.

GÖZETİM (supervision): process'lerden biri BEKLENMEDİK şekilde sonlanırsa (ör. YOLO
CUDA hatası, kamera bağlantısı kopması) bu artık sessizce fark edilmeden geçmiyor —
yüksek sesle uyarı loglanıp process yeniden başlatılıyor. Sonsuz çökme döngüsünün
"her şey yolunda" görünümü vermesini engellemek için, kısa bir zaman penceresinde
(RESTART_WINDOW_S) MAX_RESTARTS'tan fazla çökme olursa pes edilip TÜM sistem
durduruluyor — bu, kalıcı bir hatayı sonsuz yeniden başlatma arkasına gizlemez.

Kullanım: `python3 calistir.py` (klavye kısayolları için terminalin kendisi TTY
olmalı — stdio alt-process'lere miras alınır, bkz. lidar_fusion_node._keypress_listener).
"""

import os
import signal
import subprocess
import sys
import time

NODE_SCRIPTS = ['lidar_fusion_node.py', 'camera_vision_node.py']
MAX_RESTARTS = 5          # bu sayıdan fazla çökme olursa pes edilir (kalıcı hata varsayımı)
RESTART_WINDOW_S = 60.0   # sadece bu süre içindeki çökmeler sayılır (eski çökmeler unutulur)


class ManagedProcess:
    def __init__(self, script: str, base_dir: str):
        self.script = script
        self.path = os.path.join(base_dir, script)
        self.base_dir = base_dir
        self.proc = None
        self._restart_times = []
        self.start()

    def start(self):
        print(f"[calistir] ▶️  Başlatılıyor: {self.script}")
        self.proc = subprocess.Popen([sys.executable, self.path], cwd=self.base_dir)

    def poll(self):
        return self.proc.poll()

    def handle_unexpected_exit(self) -> bool:
        """Beklenmedik çıkışı loglar ve deneme hakkı varsa yeniden başlatır.
        Deneme hakkı kalmadıysa False döner (çağıran tüm sistemi durdurmalı)."""
        now = time.time()
        self._restart_times = [t for t in self._restart_times if now - t < RESTART_WINDOW_S]
        self._restart_times.append(now)

        ret = self.proc.returncode
        if len(self._restart_times) > MAX_RESTARTS:
            print(
                f"[calistir] 🚨 KALICI HATA: {self.script} son {RESTART_WINDOW_S:.0f} sn içinde "
                f"{MAX_RESTARTS}'ten fazla çöktü (son kod={ret}) — YENİDEN BAŞLATILMIYOR. "
                f"Muhtemelen kalıcı bir hata var (donanım bağlantısı, model dosyası, vb.) — "
                f"loglara bakıp düzeltin. Tüm sistem durduruluyor."
            )
            return False

        print(
            f"[calistir] 🚨 UYARI: {self.script} beklenmedik şekilde sonlandı (kod={ret}) — "
            f"yeniden başlatılıyor ({len(self._restart_times)}/{MAX_RESTARTS} deneme, "
            f"{RESTART_WINDOW_S:.0f} sn pencerede)."
        )
        self.start()
        return True

    def stop(self, timeout: float = 5.0):
        if self.poll() is None:
            print(f"[calistir] ⏹  Durduruluyor: {self.script}")
            self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            self.proc.wait()


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    managed = [ManagedProcess(script, base_dir) for script in NODE_SCRIPTS]

    try:
        while True:
            for m in managed:
                if m.poll() is not None:
                    if not m.handle_unexpected_exit():
                        return
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[calistir] Ctrl+C alındı, düzgün kapatılıyor...")
    finally:
        for m in managed:
            m.stop()


if __name__ == '__main__':
    main()
