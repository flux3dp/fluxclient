
import argparse


def gcode_2_fcode():
    parser = argparse.ArgumentParser(description='flux fcode creator')
    parser.add_argument("-i", dest='input', type=str, required=True)
    parser.add_argument("-o", dest='output', type=str, required=True)
    parser.add_argument("--type", dest='head_type', type=str,
                        default="EXTRUDER", choices=["EXTRUDER", "LASER",
                                                     "N/A"],
                        help='Set head type')
    parser.add_argument("--cor", dest='correction', type=str,
                        default=None, choices=["A", "H", "N"],
                        help='Set correction, A=ALL, H=Do height only, N=No'
                             ' correction')
    parser.add_argument("--hel", dest='head_error_level',
                        type=int, default=None, help='Head error level')
    parser.add_argument("--fmd", dest="filament_detect", type=str,
                        default=None, choices=["Y", "N", ],
                        help='Set filament detect, only for extruder type')

    options = parser.parse_args()

    from fluxclient.fcode.g_to_f import GcodeToFcode

    ext_metadata = {}
    if options.head_error_level is not None:
        ext_metadata["HEAD_ERROR_LEVEL"] = str(options.head_error_level)
    if options.correction is not None:
        ext_metadata["CORRECTION"] = str(options.correction)
    if options.filament_detect is not None:
        ext_metadata["FILAMENT_DETECT"] = str(options.filament_detect)
    if options.head_type is not None:
        ext_metadata['HEAD_TYPE'] = str(options.head_type)

    with open(options.input, "r") as in_f:
        conv = GcodeToFcode(ext_metadata=ext_metadata)

        if options.output:
            with open(options.output, "wb") as out_f:
                conv.process(in_f, out_f)


def fcode_2_gcode():
    pass
