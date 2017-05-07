# !/usr/bin/env python3

import logging

from lxml import etree as ET  # noqa

from .laser_middleware import LaserMiddleware
from fluxclient.utils.svg_parser import SVGParser


logger = logging.getLogger(__name__)


class LaserSvg(LaserMiddleware, SVGParser):
    """
    LaserSvg class:
      generate gcode base on given svg files
    """
    def __init__(self):
        super(LaserSvg, self).__init__()
        self.reset()

    def reset(self):
        self.ext_metadata['HEAD_TYPE'] = 'LASER'
        self.ext_metadata['BACKLASH'] = 'Y'

        self.svgs = {}
        self.ready_svgs = {}

    def compute(self, name, data):
        """
        setting params
        and put in dict ready_svgs (svg without params is not ready yet)
        """
        self.ready_svgs[name] = data

    def process(self, processor, names=None, ws=None):
        self.reset_image()

        if not names:
            names = self.ready_svgs.keys()

        processor.append_comment("FLUX Laser SVG Tool")
        self.moveTo(processor, x=0, y=0, speed=5000,
                    z=self.focal_l + self.obj_height)
        progress = 0.1
        offset = 4 * len(names) / 0.98
        name_index = offset

        for name in names:
            name_index += 1
            ready_svg = self.ready_svgs[name]
            svg_data = ready_svg[0].decode('utf8')
            root = ET.fromstring(svg_data)
            view_box = list(
                map(float, root.attrib['viewBox'].replace(',', ' ').split())
            )

            progress += offset
            if ws:
                ws.send_progress('converting svg', progress)

            path_data = self.elements_to_list(root)

            progress += offset
            if ws:
                ws.send_progress('processing svg', progress)

            path_data = SVGParser.process(path_data, ready_svg[1:-3], view_box,
                                          self.radius)

            progress += offset
            if ws:
                ws.send_progress('generating fcode on svg', progress)
            for each_path in path_data:
                # flag that means extruder should move to rather than drawto
                move_to = True
                for x, y in each_path:
                    if x != '\n':
                        if not move_to:
                            self.drawTo(processor, x, y, speed=self.laser_speed)
                        else:
                            self.closeTo(processor, x, y, self.travel_speed)
                            move_to = False
                    else:
                        move_to = True
                        continue
            progress += offset
            if ws:
                ws.send_progress('preparing image', progress)
            if ready_svg[-1]:
                self.add_image(ready_svg[-1], ready_svg[-3], ready_svg[-2], *ready_svg[3:-3], thres=100)
        self.turnOff(processor)
