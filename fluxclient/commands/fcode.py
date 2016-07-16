
import argparse

PROG_DESCRIPTION = 'Flux fcode/gcode convertor.'
PROG_EPILOG = ''


def gcode_2_fcode():
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)
    parser.add_argument('-i', dest='input', type=str, required=True,
                        help='Input gcode file')
    parser.add_argument('-t', '--type', dest='head_type', type=str,
                        default='EXTRUDER', choices=['EXTRUDER', 'LASER',
                                                     'N/A'],
                        help='Set toolhead type, default is EXTRUDER')
    parser.add_argument('--cor', dest='correction', type=str,
                        default=None, choices=['A', 'H', 'N'],
                        help='Set correction, A=ALL, H=Do height only, N=No'
                             ' correction')
    parser.add_argument('--hel', dest='head_error_level',
                        type=int, default=None, help='Head error level')
    parser.add_argument('--fmd', dest='filament_detect', type=str,
                        default=None, choices=['Y', 'N'],
                        help='Set filament detect, only for extruder type')
    parser.add_argument(dest='output', type=str, required=True,
                        help='Ouput fcode file')

    options = parser.parse_args()

    from fluxclient.fcode.g_to_f import GcodeToFcode

    ext_metadata = {}
    if options.head_error_level is not None:
        ext_metadata['HEAD_ERROR_LEVEL'] = str(options.head_error_level)
    if options.correction is not None:
        ext_metadata['CORRECTION'] = str(options.correction)
    if options.filament_detect is not None:
        ext_metadata['FILAMENT_DETECT'] = str(options.filament_detect)
    if options.head_type is not None:
        ext_metadata['HEAD_TYPE'] = str(options.head_type)

    with open(options.input, 'r') as in_f:
        conv = GcodeToFcode(ext_metadata=ext_metadata)

        if options.output:
            with open(options.output, 'wb') as out_f:
                conv.process(in_f, out_f)


def fcode_2_gcode():
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)
    parser.add_argument('-i', dest='input', type=str, required=True,
                        help='Input fcode file')
    # parser.add_argument('-I', dest='--include-preview', action='store_const',
    #                     const=True, default=False,
    #                     help='Output preview images')
    parser.add_argument(dest='output', type=str, required=True,
                        help='Output gcode file')

    options = parser.parse_args()

    from fluxclient.utils.f_to_g import FcodeToGcode

    with open(options.input, "rb") as f, open(options.output, "w") as t:
        parser = FcodeToGcode()
        parser.upload_content(f.read())
        parser.f_to_g(t, include_meta=True)
