
import sys

from fluxclient.upnp import misc
from fluxclient.upnp.discover import UpnpDiscover


class DiscoverPrinter(object):
    _running_chars = ["-", "\\", "|", "/"]
    _counter = 0

    def __init__(self):
        self.discover = UpnpDiscover()
        self.found = {}

        sys.stdout.write("%-25s %-20s %-10s %-8s %-3s %s\n" %
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

    def result_callback(self, discover_instance, serial, model_id, timestemp,
                        version, has_password, ipaddrs, name, **kw):

        current = [model_id, version, has_password, ipaddrs]
        if serial in self.found and current == self.found[serial]:
            return

        self.found[serial] = current
        ipaddrs_str = (("%s/%i" % tuple(i)) for i in ipaddrs)
        sys.stdout.write("\r%-25s %-20s %-10s %-3s %-8s %s\n" %
                         (misc.uuid_to_short(serial),
                          name,
                          model_id,
                          has_password and "YES" or "NO",
                          version,
                          ", ".join(ipaddrs_str)))
        sys.stdout.flush()


def main():
    s = DiscoverPrinter()
    try:
        s.go()
    except KeyboardInterrupt:
        print("\nAbort\n")


if __name__ == "__main__":
    sys.exit(main())
