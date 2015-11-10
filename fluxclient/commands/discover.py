
from uuid import UUID
import argparse
import sys

from fluxclient.upnp import misc
from fluxclient.upnp.discover import UpnpDiscover


class DiscoverPrinter(object):
    _running_chars = ["-", "\\", "|", "/"]
    _counter = 0

    def __init__(self, uuid=None):
        self.discover = UpnpDiscover(uuid=uuid)
        self.found = {}

        sys.stdout.write("%-10s %-20s %-10s %-3s %-8s %s\n" %
                         ("Serial", "Name", "Model", "PWD", "Version",
                          "IP Address"))
        sys.stdout.write("=" * 79)
        sys.stdout.write("\n")
        sys.stdout.flush()

    def go(self):
        self.discover.discover(self.result_callback, self.loop_callback)

    def loop_callback(self, discover):
        c = self._running_chars[self._counter]
        self._counter = (self._counter + 1) % len(self._running_chars)
        sys.stdout.write("\r%s" % c)
        sys.stdout.flush()

    def result_callback(self, discover_instance, uuid, serial, model_id,
                        timestemp, version, has_password, ipaddr, name, **kw):

        current = [model_id, version, has_password, ipaddr]
        if serial in self.found and current == self.found[serial]:
            return

        self.found[serial] = current
        sys.stdout.write("\r%10s %-20s %-10s %-3s %-8s %s\n" %
                         (serial,
                          name,
                          model_id,
                          has_password and "YES" or "NO",
                          version,
                          ":".join("%s" % _ for _ in ipaddr)))
        sys.stdout.write("UUID: %s\n" % uuid.hex)
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description='Discover Flux Device')
    parser.add_argument(dest='uuid', type=str, default=None, nargs="?")
    options = parser.parse_args()

    if options.uuid:
        uuid = UUID(hex=options.uuid)
    else:
        uuid = None

    s = DiscoverPrinter(uuid=uuid)
    try:
        s.go()
    except KeyboardInterrupt:
        print("\nAbort\n")


if __name__ == "__main__":
    sys.exit(main())
