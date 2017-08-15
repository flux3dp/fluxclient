
from PIL import Image, ImageDraw
from io import BytesIO
from lxml import etree as ET  # noqa

from fluxclient.laser.laser_base import LaserBase
from fluxclient.utils.svg_parser import SVGParser
from fluxclient.parser._parser import get_all_points

class SvgImage(object):
    _preview_buf = None
    _coordinate_set = False

    buf = None
    viewbox_width = viewbox_height = None
    x1 = x2 = y1 = y2 = rotation = None

    def __init__(self, buf):
        self.set_svg(buf)

    def set_svg(self, buf):
        errors, result = SVGParser.preprocess(buf)
        if errors and errors[-1] == "EMPTY":
            raise RuntimeError("EMPTY")
        self.errors = errors
        self.buf, self.viewbox_width, self.viewbox_height = result

    def set_preview(self, preview_size, buf):
        self._preview_width, self._preview_height = preview_size
        self._preview_buf = buf

    def set_image_coordinate(self, point1, point2, rotation):
        self._coordinate_set = True

        self.x1, self.y1 = point1
        self.x2, self.y2 = point2
        self.rotation = rotation

class SvgeditorImage(SvgImage):
    def __init__(self, buf):
        self._preview_buf = None
        self._coordinate_set = False

        self.buf = buf
        self.errors = list()
        self._images = list()
        self._svgs = list()
        self.viewbox_width = self.viewbox_height = None
        self.x1 = self.x2 = self.y1 = self.y2 = self.rotation = None

        self.name_space = 'http://www.w3.org/2000/svg'
        self.xlink_name_space = 'http://www.w3.org/1999/xlink'
        self.find_image = ET.ETXPath("{%s}image" % self.name_space)
        self.run()

    @property
    def images(self):
        return self._images

    @property
    def svgs(self):
        return self._svgs

    def set_svg(self, buf, viewbox_width, viewbox_height):
        self.viewbox_width = viewbox_width
        self.viewbox_height = self.viewbox_height
        #errors, result = SVGParser.preprocess(buf)
        #if errors and errors[-1] == "EMPTY":
        #    raise RuntimeError("EMPTY")
        #self.errors = errors
        #self.buf, self.viewbox_width, self.viewbox_height = result

    def run(self):
        svg_tree = ET.XML(self.buf)
        self._separate_images(svg_tree)

    def _rename_buf(self, dic):
        dic['buf'] = dic.pop("{%s}href" % self.xlink_name_space, "")
        return dic

    def _put_into_image_space(self, images):
        self._images.extend(images)

    def _put_into_svg_space(self, svg):
        self._svgs.append(svg)

    def _gen_image_data(self, images, strong, speed):
        l = list()
        for image in images:
            dic = dict(zip(image.keys(), image.values()))
            dic['strong'] = float(strong)
            dic['speed'] = float(strong)
            dic = self._rename_buf(dic)
            l.append(dic)
        return l

    def _get_strong_and_speed(self, group):
            strong = group.attrib.get('data-strong', 0.0)
            speed = group.attrib.get('speed', 200.0)
            return strong, speed

    def _remove_images(self, group):
        images = self.find_image(group)
        for image in images:
            group.remove(image)
        return group

    def _separate_images(self, tree):
        for group in tree:
            try:
                strong, speed = self._get_strong_and_speed(group)
                element_images = self.find_image(group)
                element_group = self._remove_images(group)
                images = self._gen_image_data(element_images, strong, speed)
                self._put_into_image_space(images)
                self._put_into_svg_space(element_group)
            except ValueError:
                pass


class SvgFactory(object):
    def __init__(self, radius=150):
        self._magic = LaserBase()
        self._magic.pixel_per_mm = 10
        self._magic.radius = radius
        self._magic.shading = False

        self._svg_images = []

    @property
    def radius(self):
        return self._magic.radius

    def add_image(self, svg_image):
        self._svg_images.append(svg_image)

    def generate_preview(self):
        img = Image.new('RGBA', (100,100))

        draw = ImageDraw.Draw(img)
        draw.ellipse((25, 25, 75, 75), fill=(255, 0, 0))
        b = BytesIO()
        img.save(b, 'png')
        image_bytes = b.getvalue()
        return image_bytes

        #for img in self._svg_images:
        #    if img._preview_buf:
        #        self._magic.add_image(img._preview_buf,
        #                              img._preview_width, img._preview_height,
        #                              img.x1, img.y1, img.x2, img.y2,
        #                              img.rotation, 100)
        #return self._magic.dump(mode="preview")

    def walk(self, progress_callback=lambda p: None):
        images_length = len(self._svg_images)

        for index, image in enumerate(self._svg_images, start=1):
            progress_callback((index - 0.5) / images_length)

            svg_data = image.buf.decode('utf8')
            root = ET.fromstring(svg_data)
            viewbox = list(
                map(float, root.attrib['viewBox'].replace(',', ' ').split())
            )

        #    keys = root.attrib.keys()
        #    print('keys', keys)
#
        #    for key in keys:
        #        root.attrib.pop(key)
        #    print('after root', root.attrib)
        #    modify_svg_data = ET.tostring(root)
#
        #    print('modify_svg_data', modify_svg_data)




            svg_byte_data = str.encode(svg_data)

            path_lst = get_all_points(svg_byte_data)





        #    path_data = SVGParser.elements_to_list(root)
        #    print('path_data', len(path_data), path_data)

            progress_callback(index / images_length)

            for each_path in SVGParser.process(path_lst, (None, None,
                                                           image.x1, image.y1,
                                                           image.x2, image.y2,
                                                           image.rotation),
                                               viewbox, self.radius):
                # flag that means extruder should move to rather than drawto
                src_xy = None
                move_to = True
                for x, y in each_path:
                    if x == '\n':
                        move_to = True
                    else:
                        if move_to:
                            src_xy = (x, y)
                            move_to = False
                        else:
                            next_xy = (x, y)
                            # print("points")
                            # print((src_xy,next_xy))
                            yield src_xy, next_xy
                            src_xy = next_xy


            '''for each_path in path_lst:

                for index in range(len(each_path)-1):
                    src_xy = each_path[index]
                    next_xy = each_path[index+1]
                    print ((src_xy,next_xy))
                    yield src_xy, next_xy'''
