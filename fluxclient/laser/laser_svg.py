# !/usr/bin/env python3

import sys
from math import sin, cos, pi, sqrt
import logging
from os import environ

from lxml import etree as ET

from .laser_base import LaserBase
from fluxclient.utils.svg_parser import SVGParser


logger = logging.getLogger(__name__)


class LaserSvg(LaserBase, SVGParser):
    """
    LaserSvg class:
      generate gcode base on given svg files
    """
    def __init__(self):
        super(LaserSvg, self).__init__()
        self.reset()

    def reset(self):
        self.ext_metadata['HEAD_TYPE'] = 'LASER'

        self.svgs = {}
        self.ready_svgs = {}

    def compute(self, name, data):
        """
        setting params
        and put in dict ready_svgs (svg without params is not ready yet)
        """
        self.ready_svgs[name] = data

    def gcode_generate(self, names, ws=None):
        self.reset_image()
        gcode = []
        gcode += self.header('FLUX. Laser SVG.')
        progress = 0.1
        offset = 4 * len(names) / 0.98
        name_index = offset

        for name in names:
            name_index += 1
            ready_svg = self.ready_svgs[name]
            svg_data = ready_svg[0].decode('utf8')
            root = ET.fromstring(svg_data)
            viewBox = list(map(float, root.attrib['viewBox'].replace(',', ' ').split()))

            progress += offset
            if ws:
                ws.send_progress('converting svg', progress)

            path_data = self.elements_to_list(root)

            progress += offset
            if ws:
                ws.send_progress('processing svg', progress)

            path_data = self.process(path_data, ready_svg[1:-3], viewBox, self.radius)

            progress += offset
            if ws:
                ws.send_progress('generating fcode on svg', progress)
            for each_path in path_data:
                moveTo = True  # flag that means extruder should move to rather than drawto
                for x, y in each_path:
                    if x != '\n':
                        if not moveTo:
                            gcode += self.drawTo(x, y, speed=self.laser_speed)
                        else:
                            gcode += self.closeTo(x, y, self.travel_speed)
                            moveTo = False
                    else:
                        moveTo = True
                        continue
            progress += offset
            if ws:
                ws.send_progress('preparing image', progress)
            if ready_svg[-1]:
                self.add_image(ready_svg[-1], ready_svg[-3], ready_svg[-2], *ready_svg[3:-3], thres=100)
        gcode += self.turnOff()

        gcode = "\n".join(gcode) + "\n"
        logger.debug("generate gcode done:%d bytes" % len(gcode))

        ################ fake code ##############
        if environ.get("flux_debug") == '1':
            self.dump('./preview.png')
            with open('output.gcode', 'w') as f:
                print(gcode, file=f)
        #######################################################################

        # output only moving
        # tmp = []
        # for i in gcode:
        #     tmp.append(i)
        #     if i[:2] == 'G1':
        #         tmp.append(i)
        # return "\n".join(tmp) + "\n"

        ##########################################

        return gcode

if __name__ == '__main__':
    m_laser_svg = LaserSvg()
    filename = sys.argv[1]
    with open(filename, 'rb') as f:
        m_laser_svg.preprocess(f.read())
