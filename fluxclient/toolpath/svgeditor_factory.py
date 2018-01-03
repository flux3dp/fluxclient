#import cairosvg
import base64
import re

import numpy as np

from PIL import Image, ImageDraw, ImageEnhance
from lxml import etree as ET
from io import BytesIO
from math import floor

from fluxclient.parser._parser import get_all_points
from fluxclient.hw_profile import HardwareData
from fluxclient.toolpath import DitheringProcessor

class SvgeditorImage(object):
    def __init__(self, thumbnail, svg_data, pixel_per_mm=25, hardware='beambox'):
        self.hardware = HardwareData(hardware)
        self.pixel_per_mm = pixel_per_mm

        self.buf = svg_data
        self.errors = list()
        self._groups = list()
        self._params = list()
        self._tags = dict()
        self._definitions = dict()

        self.name_space = 'http://www.w3.org/2000/svg'
        self.xlink = 'http://www.w3.org/1999/xlink'
        self.svg_init = '<svg width="{width}" height="{height}" xmlns="{ns}" xmlns:svg="{ns}" xmlns:xlink="{xlink_ns}"/>'.format(
                            width=self.hardware.width * 10,
                            height=self.hardware.length * 10,
                            ns=self.name_space,
                            xlink_ns=self.xlink
                            )

        self._gen_tags()
        self._gen_thumbnail(thumbnail)
        #self._gen_thumbnail(svg_data)
        self.run()

    @property
    def thumbnail(self):
        if not self._thumbnail:
            self.thumbnail = None
        return self._thumbnail

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
                    'symbol',
                    'use'
                   ]

        for tag in tag_list:
            self._tags[tag] = "{%s}%s" % (self.name_space, tag)

    def _element_check(self, element, tags):
        if isinstance(tags, list):
            for tag in tags:
                if element.tag == tag:
                    return True
            return False
        else:
            return element.tag == tags

    def _tag_check(self, element):
        for key, value in self._tags.items():
            if element.tag == value:
                return str(key)
        return False

    def _analysis_group(self, group):
        def process_transform_group(element):
            nonlocal elements, processedList
            el = ET.XML(self.svg_init)
            el.append(element)
            for elem in el.iter():
                ta = self._tag_check(elem)
                processedList.insert(0, ta)
            processedList.pop()
            return el

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
                    el = process_transform_group(element)
                else:
                    continue

            elif tag is 'image':
                el = self._gen_image_data(element)

            elif tag is 'use':
                use_data = self._gen_use_data(element)
                el = ET.XML(self.svg_init)
                el.append(use_data)

            else:
                el = ET.XML(self.svg_init)
                el.append(element)

            elements.append(el)
        return elements

    def _put_into_definitions_space(self, symbol):
        _id = symbol.attrib.get('id', None)
        if not _id:
            return
        self._definitions[_id] = symbol

    def _gen_thumbnail(self, thumbnail):
        mimetype, data = thumbnail.split(b',')
        self._thumbnail = Image.open(BytesIO(base64.b64decode(data)))
        self._thumbnail.save('test.png')

    #    empty = Image.new("RGBA", (4000, 4000), "white")
#
    #    output = cairosvg.svg2png(bytestring=thumbnail)
    #    #self._thumbnail = Image.open(BytesIO(output))
    #    img = Image.open(BytesIO(output))
#
    #    draw = ImageDraw.Draw(empty)
    #    line = [0, 100, 4000, 0]
    #    draw.line(line , fill=(128,128,128), width=3)
#
    #    empty.paste(img, box=(0,0))
    #    self._thumbnail = empty
#
    #    import ipdb; ipdb.set_trace()
    #    self._thumbnail.show()

    def _gen_definitions_from_defs(self, tree):
        defs_group = tree.findall(".//{%s}defs" % self.name_space)
        for defs in defs_group:
            for element in defs:
                tag = self._tag_check(element)
                if tag is 'symbol':
                    self._put_into_definitions_space(element)
            try:
                tree.remove(defs)
            except:
                pass
        return tree

    def run(self):
        svg_tree = ET.XML(self.buf)
        svg_tree = self._gen_definitions_from_defs(svg_tree)

        for elements in svg_tree:
            tag = self._tag_check(elements)
            if tag is 'g':
                params = self._get_params(elements)
                self._put_into_params_space(params)
                group = self._analysis_group(elements)
                self._put_into_group_space(group)

    def _get_buf_and_mimetype(self, dic):
        buf_with_mimetype = dic.pop("{%s}href" % self.xlink, "")
        dic['mimetype'], dic['buf'] = buf_with_mimetype.split(',')
        return dic

    def _put_into_group_space(self, group):
        self._groups.append(group)

    def _put_into_params_space(self, params):
        self._params.append(params)

    def _analysis_matrix_attrib(self, matrix):
        a,b,c,d,e,f = re.findall("\(.*?\)", matrix[0])[0].strip("()").split(",")
        a,b,c,d,e,f = map(float, (a,b,c,d,e,f))
        return a,b,c,d,e,f

    def _analysis_transform(self, x, y, transform):
        another = list()
        matrix = list()
        params = re.findall("^.*?\)| .*\)", transform)
        for param in params:
            param = param.strip()
            lis = matrix if re.match("^matrix", param) else another
            lis.append(param)
        a,b,c,d,e,f = self._analysis_matrix_attrib(matrix)
        translate = "matrix(1, 0, 0, 1, {tx}, {ty})".format(tx=x * a, ty=y * d)
        transform = "{} {} {}".format(' '.join(another), translate, ' '.join(matrix))
        return transform

    def _cal_transform(self, element):
        transform = element.attrib.pop('transform', None)
        x = float(element.attrib.pop('x', 0))
        y = float(element.attrib.pop('y', 0))
        transform = self._analysis_transform(x, y, transform)
        element.attrib['transform'] = transform
        return element

    def _parser_symbol_to_g(self, use, symbol):
        use.tag = "{%s}g" % self.name_space
        for element in symbol:
            use.append(element)
        return use

    def _gen_use_data(self, use):
        _id = use.attrib.pop("{%s}href" % self.xlink, None)
        if _id is None: return
        _id = _id.strip('#')
        symbol = self._definitions.get(_id, None)
        if symbol is None: return
        use = self._cal_transform(use)
        use  = self._parser_symbol_to_g(use, symbol)
        return use

    def _gen_image_data(self, image):
        dic = dict(zip(image.keys(), image.values()))
        dic = self._get_buf_and_mimetype(dic)
        bitmap = BitmapImage(dic, self.pixel_per_mm)
        return bitmap

    def _get_params(self, group):
        strength = group.attrib.get('data-strength', 0.0)
        speed = group.attrib.get('data-speed', 200.0)
        strength, speed = map(float, (strength, speed))
        return strength, speed


class SvgeditorFactory(object):
    def __init__ (self, pixel_per_mm=10):
        self.pixel_per_mm =pixel_per_mm

    def add_image(self, images, params):
        self.groups = list(zip(params, images))

    def add_thumbnail(self, thumbnail):
        self._thumbnail = thumbnail

    def generate_thumbnail(self):
        b = BytesIO()
        self._thumbnail.save(b, 'png')
        image_bytes = b.getvalue()
        return image_bytes

    def _is_bitmapImage(self, image):
        boolen = True if isinstance(image, BitmapImage) else False
        return boolen

    def _gen_svg_walk_path(self, image):
        pwm = 100
        svg_data = ET.tostring(image)
        paths = get_all_points(svg_data)
        for path in paths:
            for dist_xy in path:
                dist_x, dist_y = dist_xy
                dist_x = dist_x / 10
                #====================
                #dist_x = (dist_x - 3000) / 10
                #====================
                dist_y = dist_y / 10
                yield pwm, (dist_x, dist_y)
            yield 0.0, (dist_x, dist_y)

    def _filter_threshold(self, val, threshold):
        threshold = 255 / 100 * threshold
        val = 255 if val >= threshold else 0
        return val

    def _gen_bitmap_walk_path(self, image, progress_callback):
        factory = BitmapFactory(pixel_per_mm=self.pixel_per_mm)
        factory.add_image(image)

        current_val = 0
        current_from_left = False
        for from_left, y, enum in factory.walk_spath(progress_callback):
            for x, val in enum:
                if not val:
                    if current_val != 0:
                        current_val = 0
                        if current_from_left != from_left:
                            current_from_left = from_left
                            yield -1, dict(from_left=from_left)
                        yield val, (x, y)
                    else:
                        continue
                #===============
                # x = x - 300
                #===============
                if not image.shading:
                    val = self._filter_threshold(val, image.threshold)

                val = (val / 255.0) * 100

                if val != current_val:
                    current_val = val
                    if current_from_left != from_left:
                        current_from_left = from_left
                        yield -1, dict(from_left=from_left)
                    yield val, (x, y)

            if current_from_left != from_left:
                current_from_left = from_left
                yield -1, dict(from_left=from_left)
            yield val, (x,y)

    def _gen_walk_paths(self, group, speed, progress_callback):
        for item in group:
            if self._is_bitmapImage(item):
                yield -1, dict(is_bitmap = True, shading = item.shading), 0
                for strength, args in self._gen_bitmap_walk_path(item, progress_callback):
                    if strength < 0:
                        yield -1, args, 0
                    else:
                        yield strength, speed, args
            else:
                yield -1, dict(is_bitmap = False, shading = False), 0
                for strength, dist_xy in self._gen_svg_walk_path(item):
                    yield strength, speed, dist_xy

            yield 0.0, speed, 'done'

    def walk(self, progress_callback=lambda p: None):
        self.groups.reverse()

        for params, group in self.groups:
            layer_power, speed = params
            layer_power = layer_power / 100.0
            group.reverse()
            yield -1, dict(power_limit=layer_power), 0
            for pwm, args, dist_xy in self._gen_walk_paths(
                            group, speed, progress_callback):
                yield pwm, args, dist_xy

    def walk_cal(self):
        ratio = 1 / 20
        for ptr_y in range(200, 220):
            y = round(ptr_y * ratio, 2)
            for ptr_x in range(100, 5200):
                x = round(ptr_x * ratio, 2)
                yield x, y
            yield 'line', 'line'

class BitmapImage(object):
    def __init__(self, image, pixel_per_mm=10):
        self.pixel_per_mm = pixel_per_mm
        self.ratio = self.pixel_per_mm / 10
        self.shading = False
        self._setAttrs(image)
        self._convertToInt()
        self.pil_image = Image.open(BytesIO(base64.b64decode(image['buf'])))
        self._convertAttrs()

    def _setAttrs(self, image):
        for key, value in image.items():
            if key == "data-threshold":
                self.threshold = int(value)
            elif key == "data-shading":
                self.shading = False if value == 'false' else True

            self.__setattr__(key, value)

    def _convertToInt(self):
        self.width = round(int(round(float(self.width))) * self.ratio)
        self.height = round(int(round(float(self.height))) * self.ratio)
        self.x = round(int(round(float(self.x))) * self.ratio)
        self.y = round(int(round(float(self.y))) * self.ratio)

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
            self.rotate_cx, self.rotate_cy = map(
                    lambda x: x * self.ratio, (self.rotate_cx, self.rotate_cy))
        else:
            self.rotate = self.rotate_cx = self.rotate_cy = 0

    def _convertAttrs(self):
        self._convertTransform()
        self._convertRotate()


class BitmapFactory(object):
    def __init__(self, pixel_per_mm=10):
        self._image = None
        self.pixel_per_mm = pixel_per_mm
        self.dithering_processor = DitheringProcessor()

    def _clear_workspace(self):
        self._workspace = None

    def add_image(self, bitmap_image):
        self._clear_workspace()
        self._image = bitmap_image

    def _get_width_length(self, hardware):
        if hardware.plate_shape is 'rectangular':
            width = round(hardware.width * self.pixel_per_mm)
            length = round(hardware.length * self.pixel_per_mm)
        elif hardware.plate_shape is 'elliptic':
            width = length = round(hardware.radius * self.pixel_per_mm)
        return width, length

    def _gen_empty_workspace(self):
        hardware = HardwareData('beambox')
        width, length = self._get_width_length(hardware)
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
        bbox = (round(img_center[0] - center[0]), round(img_center[1] - center[1]))
        bbox += (round(img_center[0] + center[0]), round(img_center[1] + center[1]))
        return bbox
    
    def _floyd_steinberg_dither(self, image, progress_callback = lambda p: None):
        """
        https://en.wikipedia.org/wiki/Floyd–Steinberg_dithering
        Pseudocode:
        for each y from top to bottom
           for each x from left to right
              oldpixel  := pixel[x][y]
              newpixel  := find_closest_palette_color(oldpixel)
              pixel[x][y]  := newpixel
              quant_error  := oldpixel - newpixel
              pixel[x+1][y  ] := pixel[x+1][y  ] + quant_error * 7/16
              pixel[x-1][y+1] := pixel[x-1][y+1] + quant_error * 3/16
              pixel[x  ][y+1] := pixel[x  ][y+1] + quant_error * 5/16
              pixel[x+1][y+1] := pixel[x+1][y+1] + quant_error * 1/16
        find_closest_palette_color(oldpixel) = floor(oldpixel / 256)
        """

        data = np.array(image)

        progress_callback("Dithering", 0.5)

        self.dithering_processor.dither(data)

        return Image.fromarray(data, 'RGBA')

    def _get_workspace(self, progress_callback = lambda p: None):
        if self._workspace:
            return self._workspace

        workspace = self._gen_empty_workspace()
        img = self._image

        resized_img = img.pil_image.resize((img.width, img.height))
        print("Resizing image - ", img.width, img.height)
        rotated_img = self._rotate_img(resized_img, -img.rotate)
        if img.shading:
            print("Dithering image - ", img.width, img.height)
            rotated_img = self._floyd_steinberg_dither(rotated_img, progress_callback)
        print("Calculating Image - ", img.width, img.height)
        workspace.imgbbox = self._cal_corner(img, rotated_img)
        workspace.paste(rotated_img, box=workspace.imgbbox[:2])
       # workspace.save('workspace.png', 'JPEG')

        self._workspace = workspace
        return workspace

    def walk_spath(self, progress_callback):
        fromLeft = True
        def find_the_bbox(box, workspace):
            left, upper, right, lower = box
            left = 0 if left < 0 else left
            upper = 0 if upper < 0 else upper
            right = workspace.width - 1 if right > workspace.width else right
            lower = workspace.height - 1 if lower > workspace.height else lower
            return (left, upper, right, lower)

        def x_enum(row):
            nonlocal fromLeft
            if not fromLeft:
                fromLeft = True
                for pixelX in range(right , left, -1 ):
                    x = round(pixelX * ratio, 2)
                    val = 255 - workspace.getpixel((pixelX, row))
                    yield x, val

            else:
                fromLeft = False
                for pixelX in range(left, right):
                    x = round(pixelX * ratio, 2)
                    val = 255 - workspace.getpixel((pixelX, row))
                    yield x, val

        ratio = 1 / self.pixel_per_mm
        workspace = self._get_workspace(progress_callback).convert("L")
        #workspace = self._get_workspace().convert("1")
        left, upper, right, lower = self._get_workspace().imgbbox
        left, upper, right, lower = find_the_bbox(
                                        (left, upper, right, lower), workspace)
        workspace.save("workspace.jpg", "JPEG")

        for ptr_y in range(upper, lower):
            progress_callback("Calculating Toolpath - " + str(round( (upper - ptr_y) * 100 / (upper - lower) )) + "%", (upper - ptr_y) / (upper - lower))
            #progress = ptr_y / workspace.height
            y = round(ptr_y * ratio, 2)
            #yield progress, y, x_enum(ptr_y)
            yield fromLeft, y, x_enum(ptr_y)

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
            y = round(ptr_y * ratio, 2)
            #yield progress, y, x_enum(ptr_y)
            yield y, x_enum(ptr_y)

if __name__ == "__main__":
    import ipdb; ipdb.set_trace()
    pass
