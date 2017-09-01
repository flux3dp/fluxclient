import base64
import re

from PIL import Image, ImageDraw, ImageEnhance
from lxml import etree as ET
from io import BytesIO

from fluxclient.parser._parser import get_all_points
from fluxclient.hw_profile import HardwareData

class SvgeditorImage(object):
    def __init__(self, buf, pixel_per_mm=10, hardware='beambox'):
        self._preview_buf = None
        self.hardware = HardwareData(hardware)
        self.pixel_per_mm = pixel_per_mm

        self.buf = buf
        self.errors = list()
        self._groups = list()
        self._params = list()
        self.tags = dict()

        self.name_space = 'http://www.w3.org/2000/svg'
        self.xlink_name_space = 'http://www.w3.org/1999/xlink'
        self.svg_init = '<svg width="{width}" height="{height}" xmlns="{ns}" xmlns:svg="{ns}" xmlns:xlink="{xlink_ns}"/>'.format(
                            width=round(self.hardware.width * self.pixel_per_mm),
                            height=round(self.hardware.length * self.pixel_per_mm),
                            ns=self.name_space,
                            xlink_ns=self.xlink_name_space
                            )

        self._gen_tags()
        self.run()

    @property
    def groups(self):
        return self._groups

    @property
    def params(self):
        return self._params

    def _gen_tags(self):
        tag_list = [
                    'g',
                    'title',
                    'image',
                    'defs',
                    'path',
                    'use'
                   ]

        for tag in tag_list:
            self.tags[tag] = "{%s}%s" % (self.name_space, tag)

    def _element_check(self, element, tags):
        if isinstance(tags, list):
            for tag in tags:
                if element.tag == tag:
                    return True
            return False
        else:
            return element.tag == tags

    def _tag_check(self, element):
        for key, value in self.tags.items():
            if element.tag == value:
                return str(key)
        return False



    def _analysis_group(self, group):
        def process_transform_group(element):
            nonlocal elements, processedList
            #el = ET.XML('<svg width="3000" height="2000" xmlns="http://www.w3.org/2000/svg" xmlns:svg="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"/>')
            el = ET.XML(self.svg_init)
            el.append(element)
            for elem in el.iter():
                ta = self._tag_check(elem)
                processedList.insert(0, ta)
            processedList.pop()

        def ignore_processed_element():
            nonlocal processedList
            processedList.pop()


        elements = list()
        processedList = list()
        for element in group.iter():
            if processedList:
                ignore_processed_element()
                continue

            tag = self._tag_check(element)
            if tag is 'g' or tag is 'title':
                if element.attrib.get('transform'):
                    process_transform_group(element)
                else:
                    continue

            elif tag is 'image':
                el = self._gen_image_data(element)
            else:
                el = ET.XML(self.svg_init)
                el.append(element)
            elements.append(el)
        return elements

    def run(self):
        svg_tree = ET.XML(self.buf)
        for elements in svg_tree:
            tag = self._tag_check(elements)
            if tag is 'g':
                params = self._get_params(elements)
                self._put_into_params_space(params)
                group = self._analysis_group(elements)
                self._put_into_group_space(group)

    def _get_buf_and_mimetype(self, dic):
        buf_with_mimetype = dic.pop("{%s}href" % self.xlink_name_space, "")
        dic['mimetype'], dic['buf'] = buf_with_mimetype.split(',')
        return dic

    def _put_into_group_space(self, group):
        self._groups.append(group)

    def _put_into_params_space(self, params):
        self._params.append(params)

    def _gen_image_data(self, image):
        dic = dict(zip(image.keys(), image.values()))
        dic = self._get_buf_and_mimetype(dic)
        bitmap = BitmapImage(dic)
        return bitmap

    def _get_params(self, group):
        strength = group.attrib.get('data-strength', 0.0)
        speed = group.attrib.get('data-speed', 200.0)
        shading = group.attrib.get('data-shading', False)
        strength, speed = map(float, (strength, speed))
        shading = False if shading == 'false' else True
        return strength, speed, shading


class SvgeditorFactory(object):
    def add_image(self, images, params):
        self.groups = list(zip(params, images))

    def generate_preview(self):
        #TODO
        img = Image.new('RGBA', (100,100))

        draw = ImageDraw.Draw(img)
        draw.ellipse((25, 25, 75, 75), fill=(255, 0, 0))
        b = BytesIO()
        img.save(b, 'png')
        image_bytes = b.getvalue()
        return image_bytes

    def _is_bitmapImage(self, image):
        boolen = True if isinstance(image, BitmapImage) else False
        return boolen

    def _gen_svg_walk_path(self, image, strength, shading):
        pwm = (255 / 100) * strength
        svg_data = ET.tostring(image)
        paths = get_all_points(svg_data)
        for path in paths:
            for dist_xy in path:
                #====================
                #dist_x, dist_y = dist_xy
                #dist_x = (dist_x - 3000) / 10
                #dist_y = dist_y / 10
                #====================
                yield pwm, (dist_x, dist_y)
            yield 0.0, (dist_x, dist_y)

    def _filter_threshold(self, val, threshold):
        threshold = 255 / 100 * threshold
        val = 255 if val >= threshold else 0
        return val

    def _gen_bitmap_walk_path(self, image, strength, shading):
        def different_val(val):
            nonlocal current_val
            if val != current_val:
                current_val = val
                return True
            return False

        factory = BitmapFactory()
        factory.add_image(image)
        for y, enum in factory.walk_spath():
            current_val = 0
            for x, val in enum:
                if not val:
                    continue
                #===============
                # x = x - 300
                #===============
                if not shading:
                    val = self._filter_threshold(val, image.threshold)

                val = (val / 100) * strength
                if different_val(val):
                    yield val, (x, y)
            yield 0, 'line'

    def _gen_walk_paths(self, group, strength, speed, shading):
        for image in group:
            if self._is_bitmapImage(image):
                walk_path_method = self._gen_bitmap_walk_path
            else:
                walk_path_method = self._gen_svg_walk_path

            for stren, dist_xy in walk_path_method(image, strength, shading):
                yield stren, speed, dist_xy

            yield 0.0, speed, 'done'

    def walk(self, progress_callback=lambda p: None):
        self.groups.reverse()

        for params, group in self.groups:
            strength, speed, shading = params
            group.reverse()
            for strength, speed, dist_xy in self._gen_walk_paths(
                                               group, strength, speed, shading):
                yield strength, speed, shading, dist_xy

class BitmapImage(object):
    def __init__(self, image):
        self._setAttrs(image)
        self._convertToInt()
        self.pil_image = Image.open(BytesIO(base64.b64decode(image['buf'])))
        self._convertAttrs()

    def _setAttrs(self, image):
        for key, value in image.items():
            if key == "data-threshold":
                self.threshold = int(value)

            self.__setattr__(key, value)

    def _convertToInt(self):
        self.width = int(round(float(self.width)))
        self.height = int(round(float(self.height)))
        self.x = int(round(float(self.x)))
        self.y = int(round(float(self.y)))

    def _convertTransform(self):
        if not hasattr(self, 'transform'):
            return
        params = re.findall("^.*?\)| .*\)", self.transform)
        for param in params:
            param = param.strip()
            key = re.findall("^.*?\(", param)
            key = key[0].strip('()')
            value = re.findall("\(.*?\)", param)
            value = value[0]
            self.__setattr__(key, value)

    def _convertRotate(self):
        if hasattr(self, 'rotate'):
            rotate = self.rotate.strip('()')
            ro, ro_x, ro_y = re.split(" |,", rotate)
            self.rotate, self.rotate_cx, self.rotate_cy = map(float, (ro, ro_x, ro_y))
        else:
            self.rotate = self.rotate_cx = self.rotate_cy = 0

    def _convertAttrs(self):
        self._convertTransform()
        self._convertRotate()


class BitmapFactory(object):
    def __init__(self, pixel_per_mm=10):
        self._image = None
        self.pixel_per_mm = pixel_per_mm

    def _clear_workspace(self):
        self._workspace = None

    def add_image(self, bitmap_image):
        self._clear_workspace()
        self._image = bitmap_image

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

    def _cal_corner(self, img, rotated_img):
        img_center = (img.x + img.width / 2, img.y + img.height / 2)
        center = tuple(map(lambda x: x/2, rotated_img.getbbox()[2:]))
        corner = (round(img_center[0] - center[0]), round(img_center[1] - center[1]))
        return corner

    def _get_workspace(self):
        if self._workspace:
            return self._workspace

        workspace = self._gen_empty_workspace()
        img = self._image

        resized_img = img.pil_image.resize((img.width, img.height))
        rotated_img = self._rotate_img(resized_img, -img.rotate)
        corner = self._cal_corner(img, rotated_img)
        workspace.paste(rotated_img, box=corner)

        self._workspace = workspace
        return workspace

    def walk_spath(self):
        def x_enum(row):
            if row % 2 == 0 :
                for pixelX in range(workspace.width - 1 , -1, -1 ):
                    x = round(pixelX * ratio, 2)
                    val = 255 - workspace.getpixel((pixelX, row))
                    yield x, val

            else:
                for pixelX in range(workspace.width):
                    x = round(pixelX * ratio, 2)
                    val = 255 - workspace.getpixel((pixelX, row))
                    yield x, val

        ratio = 1 / self.pixel_per_mm
        workspace = self._get_workspace().convert("L")
        workspace.save("workspaceL.jpg", "JPEG")

        for ptr_y in range(workspace.height):
            #progress = ptr_y / workspace.height
            y = round((ptr_y + 1) * ratio, 2)
            #yield progress, y, x_enum(ptr_y)
            yield y, x_enum(ptr_y)

    def walk_horizon(self):
        def x_enum(row):
            for pixelX in range(workspace.width):
                x = round(pixelX * ratio, 2)
                val = 255 - workspace.getpixel((pixelX, row))
                yield x, val

        ratio = 1 / self.pixel_per_mm
        workspace = self._get_workspace().convert("L")

        for ptr_y in range(workspace.height):
            #progress = ptr_y / workspace.height
            y = (ptr_y + 1) * ratio
            #yield progress, y, x_enum(ptr_y)
            yield y, x_enum(ptr_y)

if __name__ == "__main__":
    import ipdb; ipdb.set_trace()
    pass
