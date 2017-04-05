
from getpass import getuser
import argparse
import time
import sys
import os

# from io import StringIO

PROG_DESCRIPTION = 'Flux fcode/gcode convertor.'
PROG_EPILOG = ''


def create_fcode_metadata(options):
    from fluxclient import __version__

    title = os.path.splitext(os.path.basename(options.input))[0]

    md = {
        "AUTHOR": getuser(),
        "TITLE": title,
        "SOFTWARE": "fluxclient-%s-G2F-CLI" % __version__,
        "CREATED_AT": time.strftime('%Y-%m-%dT%H:%M:%SZ',
                                    time.localtime(time.time())),
    }

    if options.head_error_level is not None:
        md['HEAD_ERROR_LEVEL'] = str(options.head_error_level)
    if options.correction is not None:
        md['CORRECTION'] = str(options.correction)
    if options.filament_detect is not None:
        md['FILAMENT_DETECT'] = str(options.filament_detect)
    md['BACKLASH'] = 'Y'

    if options.preview and os.path.isfile(options.preview):
        try:
            with open(options.preview, "rb") as f:
                previews = (f.read(), )
        except Exception:
            previews = ()
    else:
        previews = ()

    return md, previews


def gcode_2_fcode(params=None, input=None, output=None):
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)
    parser.add_argument('-i', dest='input', type=str,
                        help='Input gcode file')
    parser.add_argument('-t', '--type', dest='head_type', type=str,
                        default='EXTRUDER', choices=['EXTRUDER', 'LASER',
                                                     'N/A'],
                        help='Set toolhead type, default is EXTRUDER')
    parser.add_argument('-p', '--preview', dest='preview', type=str,
                        default=None, help='Set preview image')

    parser.add_argument('--cor', dest='correction', type=str,
                        default=None, choices=['A', 'H', 'N'],
                        help='Set correction, A=ALL, H=Do height only, N=No'
                             ' correction')
    parser.add_argument('--hel', dest='head_error_level',
                        type=int, default=None, help='Head error level')
    parser.add_argument('--fmd', dest='filament_detect', type=str,
                        default=None, choices=['Y', 'N'],
                        help='Set filament detect, only for extruder type')

    parser.add_argument(dest='output', type=str,
                        help='Ouput fcode file')

    options = parser.parse_args(params)

    from fluxclient.toolpath import GCodeParser, FCodeV1FileWriter

    md, previews = create_fcode_metadata(options)

    parser = GCodeParser()
    processor = FCodeV1FileWriter(
        options.output, options.head_type, md, previews)
    parser.set_processor(processor)
    parser.parse_from_file(options.input)
    processor.terminated()

    errors = processor.errors()
    if errors:
        for err in errors:
            sys.stderr.write(err.decode())
            sys.stderr.write("\n")


def fcode_2_gcode(params=None, input=None, output=sys.stdout):
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)
    parser.add_argument('-i', dest='input', type=str,
                        help='Input fcode file')
    parser.add_argument('-p', '--unpack-preview', dest="unpack_preview",
                        action='store_const', const=True, default=False,
                        help='Output preview images')
    parser.add_argument(dest='output', type=str,
                        help='Output gcode file')

    options = parser.parse_args(params)

    from fluxclient.toolpath import FCodeParser, GCodeFileWriter

    processor = GCodeFileWriter(options.output)
    metadata, previews = FCodeParser.from_file(options.input, processor)

    if options.unpack_preview:
        if previews:
            basename, _ = os.path.splitext(options.output)
            with open(basename + ".jpg", "wb") as f:
                f.write(previews[0])
        else:
            sys.stderr.write("No previews to unpack\n")
    processor.terminated()
