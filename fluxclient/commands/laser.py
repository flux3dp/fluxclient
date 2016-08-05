
from argparse import ArgumentParser, RawTextHelpFormatter
import sys
import os
from math import sqrt

from PIL import Image

from .misc import setup_logger

PROG_DESCRIPTION = "Create laser GCode"
PROG_EPILOG = """
flux_laser usage example:
===============================================================================
\033[1ma) Convert single bitmap (Simple)\033[0m
$ flux_laser /my/image1.jpg

-------------------------------------------------------------------------------
\033[1mb) Convert single bitmap (With coordinate adjust)\033[0m
$ flux_laser @1,2,3,4,5,/my/image2.jpg

Note: '@' is required prefix, 1: is X1, 2: Y1, 3: X2, 4, Y2, 5: Rotate

-------------------------------------------------------------------------------
\033[1mc) Process multi image\033[0m
$ flux_laser @1,2,3,4,5,/my/image1.jpg /my/image2.jpg

Note: Process image1.jpg with coordinate variable, then process image2.jpg with
default
"""

logger = None


def process_svg(options, stream):
    from fluxclient.laser.laser_svg import LaserSvg
    m_laser_svg = LaserSvg()
    count = 0
    for arg in options.images:
        name = str(count)
        if arg.startswith('@'):
            try:
                w, h, x1, y1, x2, y2, rotation, filename = arg[1:].split(",")
                filename = os.path.expanduser(filename)
                w, h = float(w), float(h)
                x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
                rotation = float(rotation)

            except Exception:
                logger.error("""SVG Image argument error, syntax:
    '@width,height,x1,y1,x2,y2,r,/your/image/file/path'
    Example: 50.2,40.2,-40.2,-30.2,3.1415,/home/flux/myimage.jpg""")
                logger.exception("Argument Error")
        else:
            filename = os.path.expanduser(arg)
            w, h = 100., 100.
            x1, y1 = -50., 50.
            x2, y2 = 50., -50.
            rotation = 0.
        with open(filename, 'rb') as f:
            buf = f.read()

        m_laser_svg.svgs[name] = m_laser_svg.preprocess(buf)
        tmp_buf, tmp_w, tmp_h = m_laser_svg.svgs[name][1]

        m_laser_svg.compute(
            name + '_ready',
            [tmp_buf, w, h, x1, y1, x2, y2, rotation, 0, 0, b''])
        count += 1

    m_laser_svg.export_to_stream(stream, [name + '_ready'])


def process_to_gray_bitmap(image):
    # image is a PIL image object
    image = image.convert('L')
    return image.tobytes()


def process_bitmaps(options, stream):
    from fluxclient.laser.laser_bitmap import LaserBitmap
    from fluxclient.hw_profile import HW_PROFILE

    lb = LaserBitmap()

    for arg in options.images:
        if arg.startswith('@'):
            try:
                x1, y1, x2, y2, r, filename = arg[1:].split(",")
                filename = os.path.expanduser(filename)
                i = Image.open(filename)
                w, h = i.size
                lb.add_image(process_to_gray_bitmap(i), w, h,
                             float(x1), float(y1), float(x2), float(y2),
                             float(r), thres=options.threshold)
            except Exception:
                logger.error("Bitmap Image argument error, syntax:")
                logger.error("'@x1,y1,x2,y2,r,/your/image/file/path'")
                logger.error("Example: 50.2,40.2,-40.2,-30.2,3.1415,"
                             "/home/flux/myimage.jpg")
                logger.exception("Argument Error")
        else:
            filename = os.path.expanduser(arg)
            i = Image.open(arg)
            w, h = i.size
            wh_realworld = [w / 472 * 100, h / 472 * 100]  # dpi 120

            tmp_index = 0 if wh_realworld[0] > wh_realworld[1] else 1

            # shrink the image if needed
            diagonal = sqrt(wh_realworld[0] ** 2 + wh_realworld[1] ** 2)
            if diagonal * HW_PROFILE['model-1']['radius']:
                # * 0.96 because pre-move in front of each row
                wh_realworld[tmp_index] *= 2 * HW_PROFILE['model-1']['radius'] / diagonal * 0.96
                wh_realworld[1 - tmp_index] *= 2 * HW_PROFILE['model-1']['radius'] / diagonal * 0.96

            buf = process_to_gray_bitmap(i)
            # print(w, h, len(buf), file=sys.stderr)
            lb.add_image(buf,
                         w, h, wh_realworld[0] / -2, wh_realworld[1] / 2, wh_realworld[0] / 2, wh_realworld[1] / -2, .0,
                         thres=options.threshold)
    lb.export_to_stream(stream)


def main(params=sys.argv[1:]):
    global logger

    parser = ArgumentParser(description=PROG_DESCRIPTION,
                            formatter_class=RawTextHelpFormatter,
                            epilog=PROG_EPILOG)

    # Mode options
    mode_args = parser.add_mutually_exclusive_group()
    mode_args.add_argument('-b', dest='mode', action='store_const',
                           const="bitmap", help='Bitmap mode')
    mode_args.add_argument('-s', dest='mode', action='store_const',
                           const="svg", help='Svg mode')
    parser.add_argument('-t', dest='threshold', type=int, default=128,
                        help='Threshold for bitmap, default: 128')
    parser.add_argument('--verbose', dest='verbose', action='store_const',
                        const=True, default=False, help='Verbose output')
    parser.add_argument('-o', dest='output', type=str, default=None,
                        help='Output gcode name', required=False)
    parser.add_argument(dest='images', type=str, help='Image files',
                        nargs='+')

    options = parser.parse_args(params)
    logger = setup_logger(__name__, debug=options.verbose)

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
