
import argparse



def conv():
    parser = argparse.ArgumentParser(description='flux fcode creator')
    parser.add_argument("-i", dest='input', type=str, required=True)
    parser.add_argument("-o", dest='output', type=str, required=True)

    options = parser.parse_args()

    from fluxclient.fcode.g_to_f import GcodeToFcode


    with open(options.input, "r") as in_f:
        conv = GcodeToFcode()

        if options.output:
            with open(options.output, "wb") as out_f:
                conv.process(in_f, out_f)
