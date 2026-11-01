#!/usr/bin/env python3
"""
Njord YEDEK Sistemi — Çalıştırma Dosyası
primary/calistir.py'nin tek-node'lu sadeleştirilmiş hali: burada YALNIZCA
`yedek_lidar_node.py` (kamerasız LiDAR+pusula gate node'u) çalışır — kamera process'i
YOK, bu yüzden iki process arasında koordinasyona gerek yok. Yine de `subprocess.Popen`
ile AYRI bir process olarak başlatılır ve GÖZETİM (supervision) altındadır: node
beklenmedik şekilde çökerse (ör. Velodyne bağlantısı kopması) yüksek sesle uyarı
loglanıp yeniden başlatılır. Kısa bir zaman penceresinde (RESTART_WINDOW_S) MAX_RESTARTS'
tan fazla çökme olursa pes edilip sistem durdurulur — kalıcı bir hatayı sonsuz yeniden
başlatma arkasına gizlemez (primary/calistir.py ile aynı felsefe).

Kullanım: `python3 calistir.py` (klavye kısayolları için terminalin kendisi TTY olmalı).
"""

import os
import signal
import subprocess
import sys
import time

NODE_SCRIPTS = ['yedek_lidar_node.py']
MAX_RESTARTS = 5
RESTART_WINDOW_S = 60.0


class ManagedProcess:
    def __init__(self, script: str, base_dir: str):
        self.script = script
        self.path = os.path.join(base_dir, script)
        self.base_dir = base_dir
        self.proc = None
        self._restart_times = []
        self.start()

    def start(self):
        print(f"[calistir-yedek] ▶️  Başlatılıyor: {self.script}")
        self.proc = subprocess.Popen([sys.executable, self.path], cwd=self.base_dir)

    def poll(self):
        return self.proc.poll()

    def handle_unexpected_exit(self) -> bool:
        now = time.time()
        self._restart_times = [t for t in self._restart_times if now - t < RESTART_WINDOW_S]
        self._restart_times.append(now)

        ret = self.proc.returncode
        if len(self._restart_times) > MAX_RESTARTS:
            print(
                f"[calistir-yedek] 🚨 KALICI HATA: {self.script} son {RESTART_WINDOW_S:.0f} sn içinde "
                f"{MAX_RESTARTS}'ten fazla çöktü (son kod={ret}) — YENİDEN BAŞLATILMIYOR. "
                f"Loglara bakıp düzeltin. Sistem durduruluyor."
            )
            return False

        print(
            f"[calistir-yedek] 🚨 UYARI: {self.script} beklenmedik şekilde sonlandı (kod={ret}) — "
            f"yeniden başlatılıyor ({len(self._restart_times)}/{MAX_RESTARTS} deneme, "
            f"{RESTART_WINDOW_S:.0f} sn pencerede)."
        )
        self.start()
        return True

    def stop(self, timeout: float = 5.0):
        if self.poll() is None:
            print(f"[calistir-yedek] ⏹  Durduruluyor: {self.script}")
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
        print("\n[calistir-yedek] Ctrl+C alındı, düzgün kapatılıyor...")
    finally:
        for m in managed:
            m.stop()


if __name__ == '__main__':
    main()
