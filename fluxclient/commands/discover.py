
import sys

from fluxclient import misc
from fluxclient.upnp_discover import UpnpDiscover


class DiscoverPrinter(object):
    _running_chars = ["-", "\\", "|", "/"]
    _counter = 0

    def __init__(self):
        self.discover = UpnpDiscover()
        self.found = {}

        sys.stdout.write("%-25s %-10s %-8s %-8s %s\n" %
                         ("Serial", "Model", "Password", "Version",
                          "IP Address"))
        sys.stdout.write("=" * 79)
        sys.stdout.write("\n")
        sys.stdout.flush()

    def go(self):
        self.discover.discover(self.result_callback, self.loop_callback,
                               timeout=float("INF"))

    def loop_callback(self):
        c = self._running_chars[self._counter]
        self._counter = (self._counter + 1) % len(self._running_chars)
        sys.stdout.write("\r%s" % c)
        sys.stdout.flush()

    def result_callback(self, discover_instance, serial, model_id, timestemp,
                        protocol_version, has_password, ipaddrs):

        current = [model_id, protocol_version, has_password, ipaddrs]
        if serial in self.found and current == self.found[serial]:
            return

        self.found[serial] = current
        ipaddrs_str = (("%s/%i" % tuple(i)) for i in ipaddrs)
        sys.stdout.write("\r%-25s %-10s %-8s %-8s %s\n" %
                         (misc.uuid_to_short(serial),
                          model_id,
                          has_password and "YES" or "NO",
                          protocol_version,
                          ", ".join(ipaddrs_str)))
        sys.stdout.flush()


def main():
    s = DiscoverPrinter()
    s.go()


if __name__ == "__main__":
    sys.exit(main())
