
from fluxclient import misc
from fluxclient.upnp_discover import UpnpDiscover

discover = UpnpDiscover()
found = {}


def callback(discover_instance, serial, model_id, timestemp, protocol_version,
             has_password, ipaddrs):
    current = [model_id, protocol_version, has_password, ipaddrs]
    if serial in found and current == found[serial]:
        return

    found[serial] = current
    ipaddrs_str = (("%s/%i" % tuple(i)) for i in ipaddrs)
    print("%-25s %-10s %-8s %-8s %s" % (misc.uuid_to_short(serial),
                                        model_id,
                                        has_password and "YES" or "NO",
                                        protocol_version,
                                        ", ".join(ipaddrs_str)))

def main():
    print("%-25s %-10s %-8s %-8s %s" % ("Serial", "Model", "Password", "Version",
                                        "IP Address"))
    print("=" * 79)
    discover.discover(callback)

if __name__ == "__main__":
    sys.exit(main())
