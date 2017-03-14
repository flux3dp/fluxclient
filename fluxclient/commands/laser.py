
from argparse import ArgumentParser, RawTextHelpFormatter
from math import sqrt
import getpass
import sys
import os

from PIL import Image

from .misc import setup_logger

PROG_DESCRIPTION = "Create laser GCode"
PROG_EPILOG = """
flux_laser usage example:
===============================================================================
\033[1ma) Convert single bitmap (Simple)\033[0m
$ flux_laser -z 3.0 -o image1.fc bitmap /my/image1.jpg

-------------------------------------------------------------------------------
\033[1mb) Convert single bitmap (With coordinate adjust)\033[0m
$ flux_laser -z 3.0 -o image1.fc bitmap @1,2,3,4,5,/my/image2.jpg

Note: '@' is required prefix, 1: is X1, 2: Y1, 3: X2, 4, Y2, 5: Rotate

-------------------------------------------------------------------------------
\033[1mc) Process multi image\033[0m
$ flux_laser -z 3.0 -o image.fc bitmap @1,2,3,4,5,/my/image1.jpg /my/image2.jpg

Note: Process image1.jpg with coordinate variable, then process image2.jpg with
default
"""

logger = None


def process_svg(options):
    from fluxclient.toolpath.svg_factory import SvgImage, SvgFactory

    factory = SvgFactory()
    for arg in options.images:
        if arg.startswith('@'):
            try:
                x1, y1, x2, y2, rotation, filename = arg[1:].split(",")
                filename = os.path.expanduser(filename)
                logger.info("Processing image: %r", filename)
                x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
                rotation = float(rotation)

            except Exception:
                logger.error("""SVG Image argument error, syntax:
    '@width,height,x1,y1,x2,y2,r,/your/image/file/path'
    Example: 50.2,40.2,-40.2,-30.2,3.1415,/home/flux/myimage.jpg""")
                raise
        else:
            filename = os.path.expanduser(arg)
            logger.info("Processing image: %r", filename)
            x1, y1 = -50., 50.
            x2, y2 = 50., -50.
            rotation = 0.

        with open(filename, 'rb') as f:
            buf = f.read()

        img = SvgImage(buf)
        img.set_preview((0, 0), b"")
        img.set_image_coordinate(point1=(x1, y2), point2=(x2, y2),
                                 rotation=rotation)
        factory.add_image(img)
    return factory


def process_to_gray_bitmap(filename):
    pil_image = Image.open(filename).convert('L')
    return pil_image.size[0], pil_image.size[1], pil_image.tobytes()


def process_bitmaps(options):
    from fluxclient.toolpath.bitmap_factory import BitmapImage, BitmapFactory

    factory = BitmapFactory()
    for arg in options.images:
        if arg.startswith('@'):
            try:
                x1, y1, x2, y2, r, filename = arg[1:].split(",")
                logger.info("Processing image: %r", filename)
                filename = os.path.expanduser(filename)
                w, h, buf = process_to_gray_bitmap(filename)
                image = BitmapImage(buf, size=(w, h),
                                    point1=(float(x1), float(y1)),
                                    point2=(float(x2), float(y2)),
                                    rotation=float(r),
                                    threshold=options.threshold)
                factory.add_image(image)
            except Exception:
                logger.error("Bitmap Image argument error, syntax:")
                logger.error("'@x1,y1,x2,y2,r,/your/image/file/path'")
                logger.error("Example: 50.2,40.2,-40.2,-30.2,3.1415,"
                             "/home/flux/myimage.jpg")
                raise
        else:
            filename = os.path.expanduser(arg)
            logger.info("Processing image: %r", filename)
            w, h, buf = process_to_gray_bitmap(filename)
            wh_realworld = [w / 472 * 100, h / 472 * 100]  # dpi 120

            tmp_index = 0 if wh_realworld[0] > wh_realworld[1] else 1

            # shrink the image if needed
            diagonal = sqrt(wh_realworld[0] ** 2 + wh_realworld[1] ** 2)
            if diagonal * HW_PROFILE['model-1']['radius']:
                # * 0.96 because pre-move in front of each row
                wh_realworld[tmp_index] *= 2 * HW_PROFILE['model-1']['radius'] / diagonal * 0.96  # noqa
                wh_realworld[1 - tmp_index] *= 2 * HW_PROFILE['model-1']['radius'] / diagonal * 0.96  # noqa

            image = BitmapImage(
                buf, size=(w, h),
                point1=(wh_realworld[0] / -2, wh_realworld[1] / 2),
                point2=(wh_realworld[0] / 2, wh_realworld[1] / -2),
                rotation=.0, threshold=options.threshold)
            factory.add_image(image)
    return factory


def create_processor(options, factory):
    if options.gcode:
        from fluxclient.toolpath import GCodeFileWriter
        processor = GCodeFileWriter(options.output)
    else:
        from fluxclient.toolpath import FCodeV1FileWriter
        from fluxclient import __version__

        previews = (factory.generate_preview(), )
        processor = FCodeV1FileWriter(
            options.output, "LASER",
            {"OBJECT_HEIGHT": str(options.zheight),
             "AUTHOR": getpass.getuser(),
             "SOFTWARE": "fluxclient %s laser CLI" % __version__},
            previews)
    return processor


def main(params=sys.argv[1:]):
    global logger

    parser = ArgumentParser(description=PROG_DESCRIPTION,
                            formatter_class=RawTextHelpFormatter,
                            epilog=PROG_EPILOG)

    subparsers = parser.add_subparsers(help='Select SVG or BITMAP mode',
                                       dest="mode")

    bitmap_parser = subparsers.add_parser('bitmap')
    bitmap_parser.add_argument(
        '-t', dest='threshold', type=int, default=255,
        help='Threshold for bitmap, default: 255')
    bitmap_parser.add_argument('--max-engraving-strength',
                               dest='max_engraving_strength', type=float,
                               default=1.0, help='Value = 0.0 to 1.0')
    bitmap_parser.add_argument('--disable-shading', dest='shading',
                               action='store_const', const=False, default=True,
                               help='Disable shading')

    svg_parser = subparsers.add_parser('svg')
    svg_parser.add_argument('--engraving-strength', type=float, default=1.0,
                            dest='engraving_strength',
                            help='Value = 0.0 to 1.0')

    parser.add_argument('-g', '--gcode', dest='gcode', action='store_const',
                        const=True, default=False, help='Output gcode')
    parser.add_argument('-z', dest='zheight', type=float, help='Object height')
    parser.add_argument('--travel-speed', dest='travel_speed', type=int,
                        default=2400, help='Travel speed')
    parser.add_argument('--engraving-speed', dest='engraving_speed', type=int,
                        default=2400, help='Travel speed')
    parser.add_argument('--verbose', dest='verbose', action='store_const',
                        const=True, default=False, help='Verbose output')
    parser.add_argument('-o', dest='output', type=str, default=None,
                        help='Output gcode name', required=False)
    parser.add_argument(dest='images', type=str, help='Image files',
                        nargs='+')

    options = parser.parse_args(params)
    logger = setup_logger(__name__, debug=options.verbose)

    from fluxclient.toolpath import laser

    if options.output is None:
        raise RuntimeError("TODO: stdout not implement")

    if options.mode == "svg":
        logger.debug("Process svg")
        factory = process_svg(options)
        writer = create_processor(options, factory)
        logger.info("Calculating...")
        laser.svg2laser(
            writer, factory,
            z_height=options.zheight,
            travel_speed=options.travel_speed,
            engraving_speed=options.engraving_speed,
            engraving_strength=options.engraving_strength)
        writer.terminated()
    else:
        logger.debug("Process bitmap")
        factory = process_bitmaps(options)
        writer = create_processor(options, factory)
        logger.info("Calculating...")
        laser.bitmap2laser(
            writer, factory, z_height=options.zheight,
            travel_speed=options.travel_speed,
            engraving_speed=options.engraving_speed,
            shading=options.shading,
            max_engraving_strength=options.max_engraving_strength)
        writer.terminated()
    logger.debug("Done")
