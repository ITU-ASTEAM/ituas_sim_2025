"""
map_parser.py
costmap_colored.png dosyasını okur, yeşil (kenar) ve sarı (engel) dubaların
dünya koordinatlarını döndürür.
"""

import numpy as np
from PIL import Image
from scipy import ndimage


class MapParser:
    def __init__(self, image_path: str, resolution: float, origin: list):
        """
        image_path : costmap_colored.png tam yolu
        resolution : metre/piksel (örn: 0.05)
        origin     : [x, y, yaw] harita origin (yaml'dan)
        """
        self.resolution = resolution
        self.origin_x = origin[0]
        self.origin_y = origin[1]

        img = Image.open(image_path).convert('RGB')
        self.arr = np.array(img)
        self.height = self.arr.shape[0]

        self.green_buoys = []   # kenar dubalar
        self.yellow_buoys = []  # engel dubalar

        self._parse()

    def _parse(self):
        arr = self.arr

        # Renk maskeleri
        green_mask  = (arr[:,:,1] > 150) & (arr[:,:,0] < 100) & (arr[:,:,2] < 100)
        yellow_mask = (arr[:,:,0] > 150) & (arr[:,:,1] > 150) & (arr[:,:,2] < 100)

        self.green_buoys  = self._get_centers(green_mask)
        self.yellow_buoys = self._get_centers(yellow_mask)

    def _get_centers(self, mask):
        labeled, n = ndimage.label(mask)
        centers_px = ndimage.center_of_mass(mask, labeled, range(1, n + 1))
        result = []
        for (row, col) in centers_px:
            x, y = self._px_to_world(row, col)
            result.append((x, y))
        return result

    def _px_to_world(self, row, col):
        x = col * self.resolution + self.origin_x
        y = (self.height - row) * self.resolution + self.origin_y
        return round(x, 3), round(y, 3)

    def get_green_buoys(self):
        return self.green_buoys

    def get_yellow_buoys(self):
        return self.yellow_buoys
