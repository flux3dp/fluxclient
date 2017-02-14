
from uuid import UUID
import argparse
import json
import sys

from fluxclient.device import DeviceDiscover
from .misc import CharacterRenderHelper


PROG_DESCRIPTION = "Finds FLUX Delta devices in local network."
PROG_EPILOG = ""


class ConsoleFormatter(object):
    FORMAT_STR = "\r%32s %-10s %-9s ## %s\n%32s %-10s %-10s\n"

    def __init__(self, output):
        self.char_render = CharacterRenderHelper()
        self.output = output

    def print_head(self):
        self.output.write(
            self.FORMAT_STR % ("UUID", "Serial", "PWD", "Name", "IP Addr",
                               "Version", "Model"))
        self.output.write("=" * 79 + "\n")
        self.output.flush()

    def print_device(self, device):
        self.output.write(self.FORMAT_STR % (
            device.uuid.hex, device.serial,
            device.has_password and "YES" or "NO",
            device.name, device.ipaddr, device.version, device.model_id))
        self.output.flush()

    def print_loopback(self):
        self.char_render.render(self.output)

    def print_tail(self):
        pass


class JsonFormatter(object):
    first = True

    def __init__(self, output):
        self.output = output

    def print_head(self):
        self.output.write("[")
        self.output.flush()

    def print_device(self, device):
        if self.first:
            self.first = False
        else:
            self.output.write(", ")

        buf = json.dumps(device.to_dict(serialized=True))
        self.output.write(buf)
        self.output.flush()

    def print_loopback(self):
        pass

    def print_tail(self):
        self.output.write("]")
        self.output.flush()


class FluxDeviceDiscover(object):
    specific_uuid = None
    formatter = None

    def __init__(self, uuid=None, formatter=None):
        self.formatter = formatter

        self.specific_uuid = uuid
        self.devices = set()
        self.found = {}

    def serve_forever(self, timeout):
        try:
            self.formatter.print_head()

            discover = DeviceDiscover(uuid=self.specific_uuid)
            discover.discover(self.result_callback, self.loop_callback,
                              timeout)
        finally:
            self.formatter.print_tail()

    def loop_callback(self, discover):
        self.formatter.print_loopback()

    def result_callback(self, discover_instance, device, **kw):
        self.devices.add(device)

        uuid = device.uuid
        if uuid in self.found and device.timestamp == self.found[uuid]:
            return
        else:
            self.found[device.uuid] = device.timestamp

        self.formatter.print_device(device)

        if self.specific_uuid:
            discover_instance.stop()


def main(params=None):
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)

    parser.add_argument(dest='uuid', type=UUID, default=None, nargs="?",
                        help='Find specific device only')
    parser.add_argument('-f', dest='format', type=str, choices=['json'],
                        default='console')
    parser.add_argument('-o', dest='output', type=str, default=None,
                        help='Output to file')
    parser.add_argument('-t', dest='timeout', type=float, default=float('inf'),
                        help='Discover timeout, flux_discover will quit after'
                             'timeout')

    options = parser.parse_args(params)

    if options.output:
        output = open(options.output, "w")
    else:
        output = sys.stdout

    if options.format == 'console':
        formatter = ConsoleFormatter(output)
    elif options.format == 'json':
        formatter = JsonFormatter(output)
    else:
        raise ValueError("Unknow formatter: %s" % options.format)

    s = FluxDeviceDiscover(uuid=options.uuid, formatter=formatter)
    try:
        s.serve_forever(options.timeout)
        sys.stdout.write("\n")
    except KeyboardInterrupt:
        sys.stdout.write("\nAbort\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
