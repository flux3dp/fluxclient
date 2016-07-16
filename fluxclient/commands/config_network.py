
# WARNING: this file is deprecated and will not work anymore

"""
flux_config_network is a simple tool shows how to organize and send network
configuration to fluxmonitord daemon.
"""

from time import sleep
import argparse
import logging
import sys
import re

from fluxclient.commands.misc import get_or_create_default_key
from fluxclient.upnp.misc import parse_network_config as parse_network
from fluxclient.usb.misc import is_serial_port
from fluxclient.upnp.task import UpnpTask
from fluxclient.usb.task import UsbTask, UsbTaskError

logging.basicConfig(format="%(message)s", stream=sys.stdout)
logger = logging.getLogger(__name__)


def is_uuid(input):
    return True if re.match("[0-9a-fA-F]{32}", input) else False


def try_lan_config(task, config):
    for i in range(3):
        resp = task.config_network(config)
        if resp:
            if resp.get("status") == "ok":
                return True
            else:
                raise RuntimeError("Network change failed: %s" %
                                   resp.get("message", ""))
        sleep(0.2)
    return False


def main():
    parser = argparse.ArgumentParser(description='network config')

    parser.add_argument('-d', dest='debug', action='store_const',
                        const=True, default=False, help='Print debug log')
    parser.add_argument('--ssid', dest='ssid', type=str, default=None,
                        help='SSID, example:: FLUX', required=True)
    parser.add_argument('--security', dest='security', type=str, default=None,
                        choices=['', 'WEP', 'WPA-PSK', 'WPA2-PSK'],
                        help='wifi security')
    parser.add_argument('--psk', dest='psk', type=str, default=None,
                        help='WPA-PSK')
    parser.add_argument('--wepkey', dest='wepkey', type=str, default=None,
                        help='wepkey')
    parser.add_argument('--ip', dest='ipaddr', type=str, default=None,
                        help='IP Address, example: 192.168.1.2. '
                             'If no ip given, use dhcp')
    parser.add_argument('--mask', dest='mask', type=int, default=24,
                        help='Mask, example: 24')
    parser.add_argument('--route', dest='route', type=str, default=None,
                        help='Route, example: 192.168.1.1')
    parser.add_argument('--dns', dest='ns', type=str, default=None,
                        help='Route, example: 192.168.1.1')
    parser.add_argument(dest='target', type=str,
                        help='Printer Serial ID or serial port')
    parser.add_argument('--key', dest='clientkey', type=str, default=None,
                        help='Client identify key (A RSA pem)')

    opt = parser.parse_args()
    opt.clientkey = get_or_create_default_key(options.clientkey)
    if opt.debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    method = "static" if opt.ipaddr else "dhcp"
    c = parse_network(method=method, ipaddr=opt.ipaddr, mask=opt.mask,
                      route=opt.route, ns=opt.ns,
                      ssid=opt.ssid, security=opt.security,
                      wepkey=opt.wepkey, psk=opt.psk)
    try:
        if is_uuid(opt.target):
            logger.info("Config network over LAN...")

            task = UpnpTask(opt.target, opt.clientkey)
            task.require_auth()

            if not try_lan_config(task, c):
                raise RuntimeError("Remote no response")

        elif is_serial_port(opt.target):
            logger.info("Config network over COM port...")

            task = UsbTask(opt.target)
            task.require_auth()

            ret = task.config_network(c)
            if ret != "OK":
                raise RuntimeError(ret)
        else:
            logger.error("Unknow target: %s" % opt.target)

    except RuntimeError as e:
        if opt.debug:
            raise
        logger.error("Network change failed: %s" % e.args)
        return 2

    logger.info("Network config changed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
