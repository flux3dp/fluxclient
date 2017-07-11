
from argparse import ArgumentParser
from datetime import datetime
from PIL import Image
import getpass
import sys
import os

from fluxclient.commands.misc import setup_logger

PROG_DESCRIPTION = "Create very toolpath"
PROG_EPILOG = ""
logger = None


def process_to_gray_bitmap(filename):
    pil_image = Image.open(filename).convert('L')
    return pil_image.size[0], pil_image.size[1], pil_image.tobytes()


def prepare_bitmap_factory(options):
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
    logger.info("Image ready")
    return factory


def prepare_svg_factory(options):
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
        img.set_image_coordinate(point1=(x1, y1), point2=(x2, y2),
                                 rotation=rotation)
        for error in img.errors:
            logger.warning("Ignore SVG patten: %s", error)
        factory.add_image(img)
    logger.info("Image ready")
    return factory


def create_output_processor(options, factory, toolhead, metadata):
    from fluxclient import __version__

    metadata = dict(metadata)
    metadata.update({
        "CREATED_AT": datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ'),
        "AUTHOR": getpass.getuser(),
        "SOFTWARE": "fluxclient-%s-CLI" % __version__,
        "OBJECT_HEIGHT": str(options.zheight)})

    if options.gcode:
        from fluxclient.toolpath import GCodeFileWriter
        processor = GCodeFileWriter(options.output)
        for k, v in metadata.items():
            processor.append_comment("%s: %s" % (k, v))
        return processor
    else:
        from fluxclient.toolpath import FCodeV1FileWriter

        previews = (factory.generate_preview(), )
        return FCodeV1FileWriter(options.output, toolhead, metadata, previews)


def process_laser(options):
    from fluxclient.toolpath import laser

    if options.mode == "svg":
        factory = prepare_svg_factory(options)
        processor = create_output_processor(
            options, factory, "LASER", {"BACKLASH": "Y"})
        laser.svg2laser(
            processor, factory,
            z_height=options.zheight,
            travel_speed=options.travel_speed,
            engraving_speed=options.speed,
            engraving_strength=options.engraving_strength)
        processor.terminated()
    elif options.mode == "bitmap":
        factory = prepare_bitmap_factory(options)
        processor = create_output_processor(options, factory, "LASER", {})
        laser.bitmap2laser(
            processor, factory, z_height=options.zheight,
            travel_speed=options.travel_speed,
            engraving_speed=options.speed,
            shading=options.shading,
            max_engraving_strength=options.max_engraving_strength)
        processor.terminated()
    return 0


def process_penholder(options):
    from fluxclient.toolpath.penholder import svg2drawing

    factory = prepare_svg_factory(options)
    processor = create_output_processor(
        options, factory, "N/A", {"BACKLASH": "Y"})
    svg2drawing(processor, factory, travel_speed=options.travel_speed,
                drawing_speed=options.speed,
                drawing_zheight=options.zheight,
                travel_zheight=options.zheight + options.travel_lift)
    processor.terminated()
    return 0


def process_vinyl(options):
    from fluxclient.toolpath.penholder import svg2vinyl
    metadata = {"BACKLASH": "Y"}

    if options.precut:
        try:
            str_x, str_y = options.precut.split(",")
            precut = float(str_x), float(str_y)
            metadata["PRECUT"] = options.precut
        except (TypeError, ValueError):
            logger.error("Can not parse --precut, syntax should be two number "
                         "splited by ','. For example: --precut 12.3,45.6")
            return 1
    else:
        precut = None

    factory = prepare_svg_factory(options)
    processor = create_output_processor(
        options, factory, "N/A", metadata)

    svg2vinyl(processor, factory, precut_at=precut,
              cutting_speed=options.speed, cutting_zheight=options.zheight,
              travel_speed=options.travel_speed,
              travel_zheight=options.zheight + options.travel_lift)
    processor.terminated()


def add_obj_z_arguments(parser, include_travel_lift=False,
                        default_lift_offset=3.0):
    parser.add_argument('-z', '--zheight', dest='zheight', type=float,
                        required=True, help='Object height (mm)')
    if include_travel_lift:
        parser.add_argument('--travel-lift', dest='travel_lift', type=float,
                            default=default_lift_offset,
                            help='Lift when traveling (mm)')


def add_speed_arguments(parser, working_default, travel_default=2400):
    parser.add_argument(
        '--speed', type=int, default=working_default, dest='speed',
        help='Working speed (mm/minute)')
    parser.add_argument(
        '--travel-speed', type=int, default=travel_default,
        dest='travel_speed', help='Travel speed (mm/minute)')


def add_default_arguments(parser):
    parser.add_argument('--verbose', dest='verbose', action='store_const',
                        const=True, default=False, help='Verbose output')
    parser.add_argument('-g', '--gcode', dest='gcode', action='store_const',
                        const=True, default=False, help='Output gcode')
    parser.add_argument('-i', dest='images', type=str, action='append',
                        help='Image files')
    parser.add_argument('-o', dest='output', type=str, required=True,
                        help='Output file name')


def add_laser_arguments(parser):
    subparser = parser.add_subparsers(title="Engraving type", dest="mode")
    subparser.required = True

    svg_parser = subparser.add_parser('svg')
    add_obj_z_arguments(svg_parser)
    add_speed_arguments(svg_parser, 400)
    svg_parser.add_argument('--strength', type=float, default=1.0,
                            dest='engraving_strength',
                            help='Value: 0.0 to 1.0')
    add_default_arguments(svg_parser)

    bitmap_parser = subparser.add_parser('bitmap')
    add_obj_z_arguments(bitmap_parser)
    add_speed_arguments(bitmap_parser, 400)
    bitmap_parser.add_argument(
        '-t', '--threshold', dest='threshold', type=int, default=255,
        help='Threshold for bitmap, default: 255')
    bitmap_parser.add_argument(
        '--max-engraving-strength', dest='max_engraving_strength', type=float,
        default=1.0, help='Value = 0.0 to 1.0')
    bitmap_parser.add_argument(
        '--disable-shading', dest='shading', action='store_const',
        const=False, default=True, help='Disable shading')
    add_default_arguments(bitmap_parser)


def get_argument_parser():
    parser = ArgumentParser(description=PROG_DESCRIPTION,
                            epilog=PROG_EPILOG)
    subparsers = parser.add_subparsers(title="Toolpath type", dest="type")
    subparsers.required = True

    laser_parser = subparsers.add_parser('laser')
    add_laser_arguments(laser_parser)

    penholder_parser = subparsers.add_parser('penholder')
    add_obj_z_arguments(penholder_parser, True, 5.0)
    add_speed_arguments(penholder_parser, working_default=600)
    add_default_arguments(penholder_parser)

    vinyl_parser = subparsers.add_parser('vinyl')
    add_obj_z_arguments(vinyl_parser, True, 5.0)
    add_speed_arguments(vinyl_parser, working_default=600)
    vinyl_parser.add_argument(
        '--precut', dest='precut', type=str,
        default=None, metavar="23.5,21.2",
        help='Pre cut a short line at a point to fix knife direction.')
    add_default_arguments(vinyl_parser)

    return parser


def main(params=None):
    global logger
    parser = get_argument_parser()
    options = parser.parse_args(params)
    logger = setup_logger(__name__, debug=options.verbose)

    if options.type == "laser":
        ret = process_laser(options)
    elif options.type == "penholder":
        ret = process_penholder(options)
    elif options.type == "vinyl":
        ret = process_vinyl(options)
    else:
        logger.error("Can not handle type %r", options.type)
        ret = 1
    sys.exit(ret)


if __name__ == "__main__":
    main()
