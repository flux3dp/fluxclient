#!/usr/bin/env python3

from argparse import ArgumentParser, RawTextHelpFormatter
import logging
import sys
import os

from PIL import Image

logging.basicConfig(format="\033[1;93m%(message)s\033[0m", stream=sys.stderr)
logger = logging.getLogger('')


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
                w, h, x1, y1, x2, y2, rotation = float(w), float(h), float(x1), float(y1), float(x2), float(y2), float(rotation)

            except Exception:
                logger.error("SVG Image argument error, syntax:")
                logger.error("'@width,height,x1,y1,x2,y2,r,/your/image/file/path'")
                logger.error("Example: 50.2,40.2,-40.2,-30.2,3.1415,"
                             "/home/flux/myimage.jpg")
                logger.exception("Argument Error")
        else:
            filename = os.path.expanduser(arg)
            w = 100.
            h = 100.
            x1 = -50.
            y1 = 50.
            x2 = 50.
            y2 = -50.
            rotation = 0.
        with open(filename, 'rb') as f:
            buf = f.read()

        m_laser_svg.svgs[name] = m_laser_svg.preprocess(buf)
        tmp_buf, tmp_w, tmp_h = m_laser_svg.svgs[name]

        with open('pre.svg', 'wb') as f:
            f.write(tmp_buf)

        m_laser_svg.compute(name + '_ready', [tmp_buf, w, h, x1, y1, x2, y2, rotation, 0, 0, b''])
        count += 1

    m_laser_svg.export_to_stream(stream, [name + '_ready'])


def process_to_gray_bitmap(image):
    # image is a PIL image object
    image = image.convert('L')
    return image.tobytes()


def process_bitmaps(options, stream):
    from fluxclient.laser.laser_bitmap import LaserBitmap
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
            lb.add_image(process_to_gray_bitmap(i), w, h,
                         -28.5, 14.25, 28.5, -14.25, .0, thres=options.threshold)
    lb.export_to_stream(stream)


def main():
    parser = ArgumentParser(description='Create laser GCode',
                            formatter_class=RawTextHelpFormatter,
                            epilog="""
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
""")

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


if __name__ == "__main__":
    import os
    p = os.path.abspath(__file__)
    pwd = os.path.dirname(os.path.dirname(os.path.dirname(p)))
    sys.path.insert(0, pwd)
    main()
