

import argparse
import logging
import sys

from PIL import Image

logging.basicConfig(format="\033[1;93m%(message)s\033[0m", stream=sys.stderr)
logger = logging.getLogger('')


def process_svg(options, stream):
    from fluxclient.laser.laser_svg import LaserSvg
    raise RuntimeError("Not ready... XD")


def process_to_gray_bitmap(image):
    # image is a PIL image object
    bands = image.getbands()
    if bands == ('L', ):
        return image.tobytes()
    else:
        return image.convert('L').tobytes()

def process_bitmaps(options, stream):
    from fluxclient.laser.laser_bitmap import LaserBitmap
    lb = LaserBitmap()

    for filename in options.images:
        if filename.startswith('@'):
            pass
        else:
            i = Image.open(filename)
            w, h = i.size
            lb.add_image(process_to_gray_bitmap(i), w, h,
                         -w/40, h/40, w/40, -h/40, .0, thres=options.threshold)
    lb.export_to_stream(stream)


def main():
    parser = argparse.ArgumentParser(description='Create laser GCode')

    # Mode options
    mode_args = parser.add_mutually_exclusive_group()
    mode_args.add_argument('-b', dest='mode', action='store_const',
                           const="bitmap", help='Bitmap mode')
    mode_args.add_argument('-s', dest='mode', action='store_const',
                           const="svg", help='Svg mode')

    parser.add_argument('-t', dest='threshold', type=int, default=100,
                        help='Threshold for bitmap, default: 100')

    parser.add_argument('-d', dest='debug', action='store_const',
                        const=True, default=False, help='Print debug log')
    parser.add_argument('-o', dest='output', type=str, default=None,
                        help='Output gcode name', required=False)
    parser.add_argument(dest='images', type=str, help='Image files',
                        nargs='+')

    options = parser.parse_args()

    if options.debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    if options.mode == "bitmap":
        logger.debug("Use bitmap processor")
        processer = process_bitmaps
    elif options.mode == "svg":
        logger.debug("Use svg processor")
        processer = process_svg
    else:
        logger.debug("Use bitmap processor")
        processer = process_bitmaps

    if options.output:
        with open(options.output, "w") as f:
            processer(options, f)
    else:
        processer(options, sys.stdout)
