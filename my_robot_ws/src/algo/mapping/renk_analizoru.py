#!/usr/bin/env python3
"""
Renk Analizörü - HSV tabanlı duba renk tespiti
withmap.py ile entegrasyon için tasarlandı.

Kullanım:
    from renk_analizoru import RenkAnalizoru
    analizor = RenkAnalizoru()
    renk, oran = analizor.analiz_et(frame, x1, y1, x2, y2)

Kalibrasyon:
    python3 renk_analizoru.py --kalibre
    (Mouse ile frame üzerinde gez, terminal HSV değerlerini basar)
"""

import cv2
import numpy as np
import argparse
import time


# ─────────────────────────────────────────────────────────────────
#  HSV ARALIK TABLOSU — gerçek dünya için geniş aralıklar
#
#  Strateji: Hue'yu dar tut (renk kimliği), S ve V'yi geniş tut
#  (ışık/yansıma/sis değişimlerine karşı dayanıklılık).
#
#  Kırmızı: HSV'de 0 ve 180 civarında wrap yapar → iki aralık şart.
#  Siyah: Hue önemsiz, sadece V düşük olmalı.
#
#  Kalibrasyon: python3 renk_analizoru.py --kalibre
#  Mouse'u dubaya götür → H değerini gör → ±20 etrafında ayarla.
# ─────────────────────────────────────────────────────────────────
VARSAYILAN_ARALIKLAR = {
    "red_buoy": [
        # Alt kırmızı (H:0-15)
        (np.array([0,   70,  40]), np.array([15, 255, 255])),
        # Üst kırmızı (H:160-180)
        (np.array([155, 70,  40]), np.array([180, 255, 255])),
    ],
    "yellow_buoy": [
        # H:20-35 — sarı. Üst sınırı 35'te kes, yeşille çakışmasın.
        (np.array([20, 70, 60]), np.array([35, 255, 255])),
    ],
    "green_buoy": [
        # H:36-130 — geniş yeşil. Gazebo'da sarımsı-yeşil (H~45)
        # ile mavi-yeşil (H~100) arası her tonu yakala.
        # S alt sınırı 40: soluk/yıkanmış renkleri de kapsar.
        (np.array([36, 40, 35]), np.array([130, 255, 255])),
    ],
    "black_buoy": [
        # Siyah: Hue önemsiz, V<80 (karanlık), S geniş.
        # Su yansıması bazen S'yi yükseltir, bu yüzden S'yi serbest bırak.
        (np.array([0, 0, 0]), np.array([180, 255, 80])),
    ],
}

# Hangi YOLO label'ları "duba" sayılır?
# Boşluklu ve alt çizgili versiyonların hepsi burada — hangisi gelirse gelsin yakala.
DUBA_LABELS = [
    # Boşluklu (mevcut model: 'black buoy', 'green buoy' ...)
    "black buoy", "green buoy", "orange buoy", "red buoy", "yellow buoy",
    # Alt çizgili alternatifler
    "black_buoy", "green_buoy", "orange_buoy", "red_buoy", "yellow_buoy",
    # Tek kelime
    "buoy", "red", "green", "yellow", "black", "orange",
]


class RenkAnalizoru:
    """
    YOLO bounding box'ından HSV renk analizi yaparak duba rengini belirler.

    Parametreler
    ------------
    roi_shrink : float (0.0 – 1.0)
        Bbox'un merkez oranı. 0.65 → kenarların %17.5'ini her yönden at.
        Arka plan piksellerini dışarıda bırakır.
    min_oran : float
        Bu oranın altındaki eşleşmeler "unknown_buoy" sayılır.
    araliklar : dict | None
        None verirsen VARSAYILAN_ARALIKLAR kullanılır.
    """

    def __init__(self, roi_shrink: float = 0.65,
                 min_oran: float = 0.08,
                 araliklar: dict | None = None):
        self.roi_shrink = roi_shrink
        self.min_oran = min_oran
        self.araliklar = araliklar if araliklar is not None else VARSAYILAN_ARALIKLAR

        # Morfoloji kerneli (gürültü temizleme)
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        print(f"[RenkAnalizoru] Başlatıldı | "
              f"ROI shrink={roi_shrink:.0%} | "
              f"min_oran={min_oran:.0%} | "
              f"renk sayısı={len(self.araliklar)}")

    # ------------------------------------------------------------------
    #  Dahili yardımcılar
    # ------------------------------------------------------------------

    def _roi_crop(self, frame: np.ndarray,
                  x1: int, y1: int, x2: int, y2: int) -> np.ndarray:
        """Bbox'un merkez bölgesini kırp."""
        w, h = x2 - x1, y2 - y1
        sx = int(w * (1.0 - self.roi_shrink) / 2)
        sy = int(h * (1.0 - self.roi_shrink) / 2)
        cx1, cy1 = x1 + sx, y1 + sy
        cx2, cy2 = x2 - sx, y2 - sy
        if cx2 <= cx1 or cy2 <= cy1:          # Çok küçük bbox
            return frame[y1:y2, x1:x2]
        return frame[cy1:cy2, cx1:cx2]

    def _maske_orani(self, hsv: np.ndarray, lower: np.ndarray,
                     upper: np.ndarray) -> float:
        """Belirli bir HSV aralığının piksel oranını döndür."""
        maske = cv2.inRange(hsv, lower, upper)
        maske = cv2.morphologyEx(maske, cv2.MORPH_OPEN, self._kernel)
        toplam = hsv.shape[0] * hsv.shape[1]
        return cv2.countNonZero(maske) / toplam if toplam > 0 else 0.0

    # ------------------------------------------------------------------
    #  Ana API
    # ------------------------------------------------------------------

    def _dominant_hue(self, hsv: np.ndarray) -> tuple[float, float]:
        """
        Bbox içindeki baskın Hue ve Saturation değerini bul.
        Düşük saturation piksellerini (arka plan, gri/beyaz) filtrele.
        Dönüş: (median_hue, median_saturation) — geçerli piksel yoksa (-1, 0)
        """
        # Sadece doygun pikselleri al: S>40, V>30 (siyah hariç)
        mask_renkli = (hsv[:, :, 1] > 40) & (hsv[:, :, 2] > 30)
        renkli_piksel = hsv[:, :, 0][mask_renkli]
        if len(renkli_piksel) < 5:
            return -1.0, 0.0
        # Hue 0 ile 180 wrap problemi: kırmızı için 160+ → negatife çek
        # Basit yöntem: medyan al, yeterlice doğru
        return float(np.median(renkli_piksel)), float(np.median(hsv[:, :, 1][mask_renkli]))

    def analiz_et(self, frame: np.ndarray,
                  x1: int, y1: int, x2: int, y2: int
                  ) -> tuple[str, float]:
        """
        Verilen bbox içindeki baskın rengi döndür.

        İki aşamalı strateji:
        1. HSV aralık eşleştirme (her renk için maske oranı)
        2. Eğer kazanan oran çok düşükse, dominant hue voting ile teyit et

        Dönüş: (renk_adı, oran)
            renk_adı : "red_buoy", "green_buoy" vb. — ya da "unknown_buoy"
            oran     : 0.0–1.0 arası eşleşen piksel oranı
        """
        roi = self._roi_crop(frame, x1, y1, x2, y2)
        if roi.size == 0:
            return "unknown_buoy", 0.0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ── Aşama 1: Aralık eşleştirme ──────────────────────────────
        en_iyi_renk = "unknown_buoy"
        en_iyi_oran = self.min_oran

        for renk_adi, aralik_listesi in self.araliklar.items():
            toplam_oran = 0.0
            for (lower, upper) in aralik_listesi:
                toplam_oran += self._maske_orani(hsv, lower, upper)
            toplam_oran = min(toplam_oran, 1.0)

            if toplam_oran > en_iyi_oran:
                en_iyi_oran = toplam_oran
                en_iyi_renk = renk_adi

        # ── Aşama 2: Dominant hue ile teyit / kurtarma ──────────────
        # Eğer oran zayıfsa ya da unknown ise hue voting'e bak.
        # Esas amacı: ışık kaymasında aralığı kaçıran ama renk ailesi
        # doğru olan durumları kurtarmak.
        if en_iyi_oran < 0.20:
            dom_hue, dom_sat = self._dominant_hue(hsv)
            if dom_hue >= 0:
                hue_renk = self._hue_to_renk(dom_hue, dom_sat,
                                              np.median(hsv[:, :, 2]))
                if hue_renk != "unknown_buoy":
                    # Aralık eşleştirme başarısızdı, hue voting kullan
                    return hue_renk, max(en_iyi_oran, self.min_oran + 0.01)

        return en_iyi_renk, en_iyi_oran

    def _hue_to_renk(self, hue: float, sat: float, val: float) -> str:
        """
        Tek bir medyan hue değerinden renk adı üret.
        Gerçek dünya için geniş toleranslı — ışık kaymasına karşı dayanıklı.
        """
        # Siyah: Value çok düşük
        if val < 70:
            return "black_buoy"

        # Düşük saturation → gri/beyaz → tanımsız
        if sat < 35:
            return "unknown_buoy"

        # Hue bazlı sınıflandırma
        if hue <= 15 or hue >= 155:
            return "red_buoy"
        elif 16 <= hue <= 35:
            return "yellow_buoy"
        elif 36 <= hue <= 130:
            return "green_buoy"
        else:
            return "unknown_buoy"

    def yolo_label_coz(self, yolo_label: str,
                       frame: np.ndarray,
                       x1: int, y1: int, x2: int, y2: int
                       ) -> tuple[str, float]:
        """
        YOLO label'ına bakarak ne yapacağına karar ver:

        • YOLO zaten renk bazlıysa (red_buoy, green_buoy…) doğrudan döndür.
          Ama yine de HSV ile doğrula; düşük oran varsa HSV'ye güven.
        • YOLO tek-class'lı (buoy, obstacle…) ise HSV analizi yap.
        • Duba olmayan nesneleri olduğu gibi bırak.

        Dönüş: (nihai_label, oran)
        """
        label_lower = yolo_label.lower()

        # Duba değilse dokunma
        if label_lower not in DUBA_LABELS:
            return yolo_label, 1.0

        # Her iki durumda da HSV analizi yap
        hsv_renk, hsv_oran = self.analiz_et(frame, x1, y1, x2, y2)

        # YOLO zaten renk içeriyorsa ve HSV de aynı fikirdeyse → YOLO'ya güven
        yolo_renk_iceriyor = any(r in label_lower for r in
                                 ["red", "green", "yellow", "black", "blue",
                                  "orange", "white"])
        if yolo_renk_iceriyor and hsv_oran < self.min_oran:
            # HSV belirsiz → YOLO'nun renk adını standartlaştır
            return self._standartlastir(label_lower), 0.5

        # HSV sonucu güvenilirse onu kullan (ışık kaymasına karşı daha sağlam)
        if hsv_oran >= self.min_oran:
            return hsv_renk, hsv_oran

        # Hiçbir şey bulunamadı
        return "unknown_buoy", 0.0

    @staticmethod
    def _standartlastir(label: str) -> str:
        """
        'red buoy', 'red_buoy', 'Red Buoy', 'red' → 'red_buoy'
        Boşluklu ve alt çizgili her versiyonu normalize et.
        """
        # Boşluk ve alt çizgiyi kaldır, küçük harf yap
        temiz = label.lower().replace("_", " ")
        renkler = ["red", "green", "yellow", "black", "orange", "blue", "white"]
        for r in renkler:
            if r in temiz:
                return f"{r}_buoy"
        return "unknown_buoy"

    # ------------------------------------------------------------------
    #  Kalibrasyon modu
    # ------------------------------------------------------------------

    def kalibre_calistir(self, kamera_topic: str = "/camera/image_raw"):
        """
        Canlı kamera görüntüsünde mouse'u gezdirerek HSV değerlerini oku.
        Terminalde H / S / V değerlerini basar.
        Çıkış: q tuşu.
        """
        try:
            from kamera_modulu import KameraModulu
        except ImportError:
            print("[Kalibrasyon] kamera_modulu.py bulunamadı!")
            return

        cam = KameraModulu(topic_name=kamera_topic)
        print(f"\n[Kalibrasyon] Topic: {kamera_topic}")
        print("Mouse'u duba üzerine götür → terminal H S V değerlerini basar.")
        print("Çıkış: q\n")

        frame_ref = [None]

        def mouse_cb(event, x, y, flags, _):
            if frame_ref[0] is not None and event == cv2.EVENT_MOUSEMOVE:
                hsv_img = cv2.cvtColor(frame_ref[0], cv2.COLOR_BGR2HSV)
                h, s, v = hsv_img[y, x]
                # Hangi renk aralığına giriyor?
                test = np.array([[[h, s, v]]], dtype=np.uint8)
                eslesen = []
                for radi, aralik_listesi in self.araliklar.items():
                    for (lo, hi) in aralik_listesi:
                        if cv2.inRange(test, lo, hi)[0][0] == 255:
                            eslesen.append(radi)
                            break
                eslesme_str = ", ".join(eslesen) if eslesen else "-"
                print(f"  H:{h:3d}  S:{s:3d}  V:{v:3d}  →  {eslesme_str}   ",
                      end="\r")

        cv2.namedWindow("HSV Kalibrasyon")
        cv2.setMouseCallback("HSV Kalibrasyon", mouse_cb)
        time.sleep(1.0)

        while True:
            ret, frame = cam.get_frame()
            if ret:
                frame_ref[0] = frame
                overlay = frame.copy()
                # Her renk için anlık maskeler
                hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                for radi, aralik_listesi in self.araliklar.items():
                    toplam_maske = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
                    for (lo, hi) in aralik_listesi:
                        toplam_maske |= cv2.inRange(hsv_img, lo, hi)
                    toplam_maske = cv2.morphologyEx(
                        toplam_maske, cv2.MORPH_OPEN, self._kernel)
                    konturlar, _ = cv2.findContours(
                        toplam_maske, cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
                    for k in konturlar:
                        if cv2.contourArea(k) > 100:
                            kx, ky, kw, kh = cv2.boundingRect(k)
                            bgr = {
                                "red_buoy": (0, 0, 220),
                                "green_buoy": (0, 200, 0),
                                "yellow_buoy": (0, 220, 220),
                                "black_buoy": (80, 80, 80),
                            }.get(radi, (200, 200, 200))
                            cv2.rectangle(overlay,
                                          (kx, ky), (kx+kw, ky+kh), bgr, 2)
                            cv2.putText(overlay, radi,
                                        (kx, ky - 6),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.4, bgr, 1)
                cv2.imshow("HSV Kalibrasyon", overlay)

            if cv2.waitKey(15) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()
        print("\n[Kalibrasyon] Tamamlandı.")


# ──────────────────────────────────────────────────────────────────
#  Bağımsız çalıştırma
# ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Duba Renk Analizörü")
    parser.add_argument("--kalibre", action="store_true",
                        help="HSV kalibrasyon modunu başlat")
    parser.add_argument("--topic", default="/camera/image_raw",
                        help="Kamera topic adı")
    args = parser.parse_args()

    analizor = RenkAnalizoru()

    if args.kalibre:
        analizor.kalibre_calistir(kamera_topic=args.topic)
    else:
        print("Kullanım: python3 renk_analizoru.py --kalibre [--topic /camera/image_raw]")
