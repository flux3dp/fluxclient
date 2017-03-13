
from fluxclient.laser.laser_base import LaserBase


class BitmapImage(object):
    def __init__(self, buf, size, point1, point2, rotation, threshold):
        self.buf = buf
        self.width, self.height = size
        self.x1, self.y1 = point1
        self.x2, self.y2 = point2
        self.rotation = rotation
        self.threshold = threshold


class BitmapFactory(object):
    def __init__(self, radius=85, pixel_per_mm=10):
        self._magic = LaserBase()
        self._magic.pixel_per_mm = pixel_per_mm
        self._magic.radius = radius
        self._magic.shading = True

    @property
    def pixel_per_mm(self):
        return self._magic.pixel_per_mm

    @property
    def radius(self):
        return self._magic.radius

    def add_image(self, bitmap_image):
        self._magic.add_image(bitmap_image.buf,
                              bitmap_image.width, bitmap_image.height,
                              bitmap_image.x1, bitmap_image.y1,
                              bitmap_image.x2, bitmap_image.y2,
                              bitmap_image.rotation,
                              bitmap_image.threshold)

    def generate_preview(self):
        return self._magic.dump(mode="preview")

    def walk_horizon(self):
        ratio = 1 / self.pixel_per_mm

        xlen = len(self._magic.image_map[0])
        xoffset = xlen // 2

        # xaxis is bitmap to real world mapping
        xaxis = tuple((v * ratio
                      for v in range(-xoffset, xlen - xoffset)))
        xlast = (xlen - xoffset + 1) * ratio

        def x_enum(row):
            for ptr_x, x in enumerate(xaxis):
                yield x, 255 - row[ptr_x]
            yield xlast, 0

        row_length = len(self._magic.image_map)
        ytop = row_length * ratio / 2
        for ptr_y, row in enumerate(self._magic.image_map):
            y = ytop - ptr_y * ratio
            yield (ptr_y / row_length), y, x_enum(row)
