
from math import sin, cos, tan, pi
import logging
from io import BytesIO

from PIL import Image, ImageEnhance, ImageOps
from fluxclient.hw_profile import HardwareData
from fluxclient.laser.laser_base import LaserBase
logger = logging.getLogger(__name__)

WORKSPACE_SIZE = 300  # (mm)


def recalculate_bound(x1, y1, x2, y2, rotate, image):
    rotate = rotate % pi

    if rotate < 0.0087:
        return ((x1, y1), (x2, y2), image.convert("RGBA"))
    elif rotate - (pi / 2.0) < 0.0087:
        cx, cy = ((x1 + x2) / 2.0), ((y1 + y2) / 2.0)
        x3 = cx + (y2 - y1) / 2.0
        y3 = cy + (x1 - x2) / 2.0
        x4 = cx - (y2 - y1) / 2.0
        y4 = cy - (x1 - x2) / 2.0
        return ((x3, y3), (x4, y4), image.convert("RGBA").rotate(90, expand=True))
    elif rotate - pi < 0.0087:
        return ((x1, y1), (x2, y2), image.convert("RGBA").rotate(180))
    elif rotate - (pi / 2.0 * 3.0):
        cx, cy = ((x1 + x2) / 2.0), ((y1 + y2) / 2.0)
        x3 = cx + (y2 - y1) / 2.0
        y3 = cy + (x1 - x2) / 2.0
        x4 = cx - (y2 - y1) / 2.0
        y4 = cy - (x1 - x2) / 2.0
        return ((x1, y1), (x2, y2), image.convert("RGBA").rotate(270, expand=True))
    else:
        t1 = tan(rotate)
        t2 = tan(pi - rotate)

        x3 = ((t1 * x1) - y1 - (t2 * x2) + y2) / (t1 - t2)
        y3 = t1 * x3 - t1 * x1 + y1
        x4 = ((t1 * x2) - y2 - (t2 * x1) + y1) / (t1 - t2)
        y4 = t2 * x4 - t2 * x1 + y1
        return ((min(x1, x2, x3, x4), max(y1, y2, y3, y4)),
                (max(x1, x2, x3, x4), min(y1, y2, y3, y4)),
                image.convert("RGBA").rotate(rotate / 180.0 * pi, expand=True))


class BitmapImage(object):
    def __init__(self, buf, size, point1, point2, rotation, threshold):
        self.buf = buf
        self.width, self.height = size
        self.x1, self.y1 = point1
        self.x2, self.y2 = point2
        self.rotation = rotation
        self.threshold = threshold
        self._workspace = None

    @property
    def pil_image(self):
        return Image.frombytes('L', (self.width, self.height), self.buf)
        # return Image.open(BytesIO(self.buf))

    def get_bound(self):
        cx, cy = (self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0
        r = self.rotation
        s, c = sin(-r), cos(-r)

        x1, y1 = self.x1 - cx, self.y1 - cy
        x2, y2 = self.x2 - cx, self.y2 - cy

        x1, y1 = c * x1 - s * y1, s * x1 + c * y1
        x2, y2 = c * x2 - s * y2, s * x2 + c * y2

        orig_bounds = ((x1, y1), (x2, y1), (x2, y2), (x1, y2))
        s, c = sin(r), cos(r)
        return tuple(((c * x - s * y + cx, s * x + c * y + cy)
                      for x, y in orig_bounds))


class BitmapFactory(object):
    def __init__(self, radius=150, pixel_per_mm=10):
        self._magic = LaserBase()
        self._magic.pixel_per_mm = pixel_per_mm
        self._magic.radius = radius
        self._magic.shading = True
        self._images = []

    @property
    def pixel_per_mm(self):
        return self._magic.pixel_per_mm

    @property
    def radius(self):
        return self._magic.radius

    def _clear_workspace(self):
        self._workspace = None

    def _gen_preview_image(self):
        #TODO
        pass

    def add_image(self, bitmap_image):
        self._clear_workspace()
        self._gen_preview_image()
    #    self._magic.add_image(bitmap_image.buf,
    #                          bitmap_image.width, bitmap_image.height,
    #                          bitmap_image.x1, bitmap_image.y1,
    #                          bitmap_image.x2, bitmap_image.y2,
    #                          bitmap_image.rotation,
    #                          bitmap_image.threshold)

        self._images.append(bitmap_image)

    def _crop_image(self, left, upper, right, lower):
        pass

    def _delta_image(self):
        """
        for temp using, need to be fix.
        """
        size = int(WORKSPACE_SIZE * self.pixel_per_mm)
        center = int(size / 2)
        # Create a (size, size) space
        workspace = Image.new("RGBA", (size, size), "white")

        for img in self._images:
            c1, c2, rotated_img = recalculate_bound(img.x1, img.y1, img.x2, img.y2, img.rotation, img.pil_image)

            w = abs(int((c2[0] - c1[0]) * self.pixel_per_mm))
            h = abs(int((c1[1] - c2[1]) * self.pixel_per_mm))
            alt_img = rotated_img.resize((w, h))

            box = [
                int(c1[0] * self.pixel_per_mm + center),
                int(center - c1[1] * self.pixel_per_mm),
                int(c1[0] * self.pixel_per_mm + center) + w,
                int(center - c1[1] * self.pixel_per_mm) + h,
            ]

            crop_x0, crop_y0, crop_x1, crop_y1 = 0, 0, alt_img.width, alt_img.height
            if box[0] > size or box[1] > size or box[2] < 0 or box[3] < 0:
                continue

            if box[0] < 0:
                crop_x0 = box[0] * -1
                box[0] = 0
            if box[1] < 0:
                crop_y0 = box[1] * -1
                box[1] = 0
            if box[2] > size:
                box[2] = size
                crop_x1 -= (box[2] - size)
            if box[3] > size:
                box[3] = size
                crop_y1 -= (box[2] - size)

            alt_img = alt_img.crop((crop_x0, crop_y0, crop_x1, crop_y1))
            workspace.paste(alt_img, box=box)
        return workspace

    def _get_witdh_length(self, hardware):
        if hardware.plate_shape is 'rectangular':
            width = round(hardware.width * self.pixel_per_mm)
            length = round(hardware.length * self.pixel_per_mm)
        elif hardware.plate_shape is 'elliptic':
            width = length = round(hardware.radius * self.pixel_per_mm)
        return width, length

    def _gen_empty_workspace(self):
        hardware = HardwareData('beambox')
        width, length = self._get_witdh_length(hardware)
        workspace = Image.new("RGBA", (width, length), "white")
        return workspace

    def _rotate_img(self, img, degree):
        temp_img = img.convert("RGBA").rotate(degree, expand=True)
        empty_img = Image.new('RGBA', temp_img.size, "white")
        out_img = Image.composite(temp_img, empty_img, temp_img)
        return out_img

    def _resize_img(self, img):
        temp_x1 = img.x1 * cos(img.rotation) - sin(img.rotation) * img.y1
        temp_y1 = img.x1 * sin(img.rotation) + cos(img.rotation) * img.y1
        temp_x2 = img.x2 * cos(img.rotation) - sin(img.rotation) * img.y2
        temp_y2 = img.x2 * sin(img.rotation) + cos(img.rotation) * img.y2

        h = round(abs(temp_y2 - temp_y1) * self.pixel_per_mm)
        w = round(abs(temp_x2 - temp_x1) * self.pixel_per_mm)

        resized_img = img.pil_image.resize((w, h))
        return resized_img

    def _cal_corner(self, img, rotated_img):
        img_center = (
                        ((img.x1 + img.x2) / 2) * self.pixel_per_mm,
                        ((img.y1 + img.y2) / 2) * self.pixel_per_mm
                     )

        center = tuple(map(lambda x: x/2, rotated_img.getbbox()[2:]))
        corner = (round(img_center[0] - center[0]), round(img_center[1] - center[1]))
        return corner


    def _get_workspace(self):
        if self._workspace:
            return self._workspace

        workspace = self._gen_empty_workspace()

        for img in self._images:

            #====Enhance test
            brightness = ImageEnhance.Brightness(img.pil_image)
            bright_img = brightness.enhance(2.0)
            bright_img.save("bright2.jpg")
            sharpness = ImageEnhance.Sharpness(img.pil_image)
            sharp_img = sharpness.enhance(10.0)
            sharp_img.save("sharp.jpg")
            #======

            resized_img = self._resize_img(img)
            #resized_img.save("resized.jpg", "JPEG")

            rotate = img.rotation * 180 / pi
            rotated_img = self._rotate_img(resized_img, rotate)
            #rotated_img.save("rotated.jpg", "JPEG")

            corner = self._cal_corner(img, rotated_img)
            workspace.paste(rotated_img, box=corner)
            mirror_image = ImageOps.mirror(rotated_img)
            #workspace.paste(mirror_image, box=corner)



        #========mirror test
        #mirror = ImageOps.mirror(workspace)
        #mirror.save("mirror.jpg", "JPEG")
        #============
        workspace.save("workspace.jpg", "JPEG")
        #mirror.save("workspace.jpg", "JPEG")
        self._workspace = workspace
        return workspace

    def generate_preview(self, new_ver=False):
        if new_ver:
            b = BytesIO()
            self._get_workspace().save(b, "png")
            return b.getbuffer().tobytes()
        else:
            return self._magic.dump(mode="preview")

    def walk_spath(self):
        def x_enum(row):
            if row % 2 == 0 :
                for pixelX in range(workspace.width - 1 , -1, -1 ):
                    x = round(pixelX * ratio, 2)
                    val = 255 - workspace.getpixel((pixelX, row))
                    #yield x, val
                    yield x - 300, val

            else:
                for pixelX in range(workspace.width):
                    x = round(pixelX * ratio, 2)
                    val = 255 - workspace.getpixel((pixelX, row))
                    #yield x, val
                    yield x - 300, val

        ratio = 1 / self.pixel_per_mm
        workspace = self._get_workspace().convert("L")
        #workspace.save("workspaceL.jpg", "JPEG")

        for ptr_y in range(workspace.height):
            progress = ptr_y / workspace.height
            y = (ptr_y + 1) * ratio
            yield progress, y, x_enum(ptr_y)

    def walk_horizon(self):
        def x_enum(row):
            for pixelX in range(workspace.width):
                x = round(pixelX * ratio, 2)
                val = 255 - workspace.getpixel((pixelX, row))
                yield x, val

        ratio = 1 / self.pixel_per_mm
        workspace = self._get_workspace().convert("L")

        for ptr_y in range(workspace.height):
            progress = ptr_y / workspace.height
            y = (ptr_y + 1) * ratio
            yield progress, y, x_enum(ptr_y)
