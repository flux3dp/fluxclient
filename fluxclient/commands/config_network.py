
"""
flux_config_network is a simple tool shows how to organize and send network
configuration to fluxmonitord daemon.
"""

from time import sleep
import argparse
import sys

from fluxclient.upnp.misc import parse_network_config as parse_network
from fluxclient.upnp.task import UpnpTask


def main(opt):
    parser = argparse.ArgumentParser(description='network config')

    parser.add_argument('--ip', dest='ipaddr', type=str, default=None,
                        help='IP Address, example: 192.168.1.2. '
                             'If no ip given, use dhcp')
    parser.add_argument('--mask', dest='mask', type=int, default=24,
                        help='Mask, example: 24')
    parser.add_argument('--route', dest='route', type=str, default=None,
                        help='Route, example: 192.168.1.1')
    parser.add_argument('--dns', dest='ns', type=str, default=None,
                        help='Route, example: 192.168.1.1')
    parser.add_argument('--ssid', dest='ssid', type=str, default=None,
                        help='SSID, example:: FLUX')
    parser.add_argument('--security', dest='security', type=str, default=None,
                        choices=['', 'WEP', 'WPA-PSK', 'WPA2-PSK'],
                        help='wifi security')
    parser.add_argument('--psk', dest='psk', type=str, default=None,
                        help='WPA-PSK')
    parser.add_argument('--wepkey', dest='wepkey', type=str, default=None,
                        help='wepkey')

    parser.add_argument(dest='serial', type=str, help='Printer Serial')

    opt = parser.parse_args()

    method = "static" if opt.ipaddr else "dhcp"
    serial = opt.serial

    task = UpnpTask(serial)
    task.require_auth()

    c = parse_network(method=method, ipaddr=opt.ipaddr, mask=opt.mask,
                      route=opt.route, ns=opt.ns,
                      ssid=opt.ssid, security=opt.security,
                      wepkey=opt.wepkey, psk=opt.psk)

    for i in range(3):
        resp = task.config_network(c)
        if resp:
            if resp.get("status") == "ok":
                print("Network config changed")
                return 0
            else:
                print("Network change failed: %s" % resp.get("message", ""))
                return 1
        sleep(0.2)

    print("Remote no response")
    return 2


if __name__ == "__main__":
    sys.exit(main())
